#include "legbot_exploration/pct_backend.hpp"
#include <Python.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <mutex>
#include <stdexcept>

namespace legbot_exploration {
namespace {
struct Object {
  PyObject *p;
  explicit Object(PyObject *value=nullptr):p(value) {}
  ~Object() {Py_XDECREF(p);}
  Object(const Object &)=delete;
  Object &operator=(const Object &)=delete;
};
std::string python_error() {
  PyObject *type=nullptr,*value=nullptr,*trace=nullptr;
  PyErr_Fetch(&type,&value,&trace);
  Object t(type),v(value),tb(trace),s(value?PyObject_Str(value):nullptr);
  const char *text=s.p?PyUnicode_AsUTF8(s.p):nullptr;
  return text?text:"PCT Python operation failed";
}
struct Gil {
  PyGILState_STATE state;
  Gil() {
    static std::once_flag initialized;
    std::call_once(initialized,[] {
      Py_Initialize();
      // All subsequent calls, including the first, acquire the GIL explicitly.
      PyEval_SaveThread();
    });
    state=PyGILState_Ensure();
  }
  ~Gil(){PyGILState_Release(state);}
};

std::vector<PctUnknownBlocker> read_unknown_blockers(PyObject *details,std::uint64_t version) {
  std::vector<PctUnknownBlocker> out;
  auto *rows=PyDict_GetItemString(details,"unknown_blockers");
  if(!rows) return out;
  if(!PyList_Check(rows)) throw std::runtime_error("invalid PCT unknown blockers");
  for(Py_ssize_t i=0;i<PyList_Size(rows);++i) {
    auto *row=PyList_GetItem(rows,i);
    if(!PyList_Check(row) || PyList_Size(row)!=4) throw std::runtime_error("invalid PCT blocker");
    PctUnknownBlocker blocker{{},0,version};
    for(int j=0;j<3;++j) {
      blocker.point[j]=PyFloat_AsDouble(PyList_GetItem(row,j));
      if(!std::isfinite(blocker.point[j])) throw std::runtime_error("invalid PCT blocker coordinate");
    }
    blocker.slice=static_cast<int>(PyLong_AsLong(PyList_GetItem(row,3)));
    if(PyErr_Occurred()) throw std::runtime_error(python_error());
    out.push_back(blocker);
  }
  return out;
}

}
struct PctBackend::Impl {
  std::string root,error,settings_json,metadata="{}";
  double height;
  std::size_t max_surfaces,max_cells,surface_records=0;
  std::vector<float> xyz;
  PyObject *backend=nullptr;
  bool initialization_failed=false;
  std::string initialization_error;
  Impl(std::string path,double h,std::size_t records,std::size_t cells,std::string settings)
    :root(std::move(path)),settings_json(std::move(settings)),height(h),max_surfaces(records),max_cells(cells) {}
  ~Impl() {if(backend && Py_IsInitialized()){Gil gil;Py_DECREF(backend);}}
  void ensure() {
    if(initialization_failed) throw std::runtime_error(initialization_error);
    if(backend) return;
    try {
    // A project-local optional runtime keeps CuPy/CUDA wheels out of system
    // Python and other ROS nodes. Look next to the installed/source workspace.
    for(auto directory=std::filesystem::path(root);!directory.empty();) {
      const auto runtime=directory/".pct_runtime";
      if(std::filesystem::is_directory(runtime)) {
        Object extra(PyUnicode_FromString(runtime.c_str()));
        if(PyList_Insert(PySys_GetObject("path"),0,extra.p)<0) throw std::runtime_error(python_error());
        break;
      }
      const auto parent=directory.parent_path();
      if(parent==directory) break;
      directory=parent;
    }
    const auto scripts=std::filesystem::path(root)/"planner"/"scripts";
    Object pct_path(PyUnicode_FromString(scripts.c_str()));
    if(!pct_path.p || PyList_Insert(PySys_GetObject("path"),0,pct_path.p)<0)
      throw std::runtime_error(python_error());
    Object path(PyUnicode_FromString(EXPLORATION_PYTHON_DIR));
    if(PyList_Insert(PySys_GetObject("path"),0,path.p)<0) throw std::runtime_error(python_error());
    Object module(PyImport_ImportModule("legbot_exploration.online_pct"));
    if(!module.p) throw std::runtime_error(python_error());
    Object type(PyObject_GetAttrString(module.p,"OnlinePct"));
    if(!type.p) throw std::runtime_error(python_error());
    backend=PyObject_CallFunction(type.p,"sdK",root.c_str(),height,
                                  static_cast<unsigned long long>(max_cells));
    if(!backend) throw std::runtime_error(python_error());
    Object json(PyImport_ImportModule("json"));
    Object settings(json.p?PyObject_CallMethod(json.p,"loads","s",settings_json.c_str()):nullptr);
    Object limit(PyLong_FromSize_t(max_surfaces));
    if(!settings.p || PyDict_SetItemString(settings.p,"max_surface_records",limit.p)<0)
      throw std::runtime_error(python_error());
    Object configured(PyObject_CallMethod(backend,"configure","O",settings.p));
    if(!configured.p) throw std::runtime_error(python_error());

    } catch(const std::exception &e) {
      initialization_failed=true;
      initialization_error=e.what();
      throw;
    }
  }
  PyObject *call(const char *method,std::uint64_t version,const Point &start,
                 const Point *target=nullptr,std::optional<XY> floor=std::nullopt,
                 std::optional<double> heading=std::nullopt,std::optional<double> platform_reference=std::nullopt) {
    ensure();
    Object buffer(PyMemoryView_FromMemory(reinterpret_cast<char*>(xyz.data()),
        static_cast<Py_ssize_t>(xyz.size()*sizeof(float)),PyBUF_READ));
    if(!buffer.p) throw std::runtime_error(python_error());
    PyObject *answer=target && floor?PyObject_CallMethod(backend,method,"OK(ddd)(ddd)(dd)",
        buffer.p,static_cast<unsigned long long>(version),start[0],start[1],start[2],
        (*target)[0],(*target)[1],(*target)[2],(*floor)[0],(*floor)[1]):
        target && heading?PyObject_CallMethod(backend,method,"OK(ddd)(ddd)d",
        buffer.p,static_cast<unsigned long long>(version),start[0],start[1],start[2],
        (*target)[0],(*target)[1],(*target)[2],*heading):
        target?PyObject_CallMethod(backend,method,"OK(ddd)(ddd)",
        buffer.p,static_cast<unsigned long long>(version),start[0],start[1],start[2],
        (*target)[0],(*target)[1],(*target)[2]):
        floor && heading && platform_reference?PyObject_CallMethod(backend,method,"OK(ddd)(dd)dd",buffer.p,
        static_cast<unsigned long long>(version),start[0],start[1],start[2],(*floor)[0],(*floor)[1],*heading,*platform_reference):
        floor && heading?PyObject_CallMethod(backend,method,"OK(ddd)(dd)d",buffer.p,
        static_cast<unsigned long long>(version),start[0],start[1],start[2],(*floor)[0],(*floor)[1],*heading):
        floor?PyObject_CallMethod(backend,method,"OK(ddd)(dd)",buffer.p,
        static_cast<unsigned long long>(version),start[0],start[1],start[2],(*floor)[0],(*floor)[1]):
        heading?PyObject_CallMethod(backend,method,"OK(ddd)d",buffer.p,
        static_cast<unsigned long long>(version),start[0],start[1],start[2],*heading):
        PyObject_CallMethod(backend,method,"OK(ddd)",buffer.p,
        static_cast<unsigned long long>(version),start[0],start[1],start[2]);
    if(!answer) throw std::runtime_error(python_error());
    return answer;
  }
};
PctBackend::PctBackend(std::string root,double h,std::size_t records,std::size_t cells,std::string settings)
  :impl_(std::make_unique<Impl>(std::move(root),h,records,cells,std::move(settings))) {}
PctBackend::~PctBackend()=default;
void PctBackend::append(const std::vector<LayerPoint> &points,std::optional<Point> sensor_origin) {
  Gil gil;
  try {
    impl_->ensure();
    auto &xyz=impl_->xyz;xyz.clear();xyz.reserve(points.size()*3);
    for(const auto &p:points) {
      if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))
        xyz.insert(xyz.end(),{static_cast<float>(p.x),static_cast<float>(p.y),static_cast<float>(p.z)});
    }
    Object buffer(PyMemoryView_FromMemory(reinterpret_cast<char*>(xyz.data()),
        static_cast<Py_ssize_t>(xyz.size()*sizeof(float)),PyBUF_READ));
    Object answer(buffer.p?(sensor_origin?
        PyObject_CallMethod(impl_->backend,"integrate","O(ddd)",buffer.p,(*sensor_origin)[0],(*sensor_origin)[1],(*sensor_origin)[2]):
        PyObject_CallMethod(impl_->backend,"integrate","O",buffer.p)):nullptr);
    if(!answer.p) throw std::runtime_error(python_error());
    impl_->surface_records=PyLong_AsSize_t(answer.p);
    if(PyErr_Occurred()) throw std::runtime_error(python_error());
    impl_->error.clear();
  }catch(const std::exception &e){PyErr_Clear();impl_->error=e.what();}
  // Release the raw frame; map readers receive an empty buffer.
  std::vector<float>().swap(impl_->xyz);
}

bool PctBackend::refresh(std::uint64_t version) {
  Gil gil;
  try {
    Object answer(impl_->call("commit_pending",version,Point{}));
    Object metrics(PyObject_GetAttrString(impl_->backend,"metadata"));
    Object json(PyImport_ImportModule("json"));
    Object encoded(metrics.p && json.p?PyObject_CallMethod(json.p,"dumps","O",metrics.p):nullptr);
    if(!encoded.p) throw std::runtime_error(python_error());
    impl_->metadata=PyUnicode_AsUTF8(encoded.p);
    impl_->error.clear();return true;
  }catch(const std::exception &e){impl_->error=e.what();return false;}
}
PctResult PctBackend::plan(std::uint64_t version,const Point &start,const Point &target,bool explicit_surface,
    std::optional<XY> floor,bool collect_blockers,bool stair_segment) {
  return plan_with_method(stair_segment?"plan_stair_surface":
      explicit_surface?(collect_blockers?"plan_surface_with_blockers":"plan_surface"):"plan",
      version,start,target,floor);
}
PctResult PctBackend::plan_shift(std::uint64_t version,const Point &start,const Point &target) {
  return plan_with_method("plan_shift",version,start,target);
}
PctResult PctBackend::plan_stair_reverse(std::uint64_t version,const Point &start,const Point &target) {
  return plan_with_method("plan_stair_reverse",version,start,target);
}
std::vector<PctObservationPose> PctBackend::observation_poses(std::uint64_t version,const Point &base,
    const Point &attitude,const std::vector<Point> &targets,std::optional<bool> include_moves,
    std::optional<double> ascent,std::optional<XY> floor,std::optional<Point> stance,
    double tolerance,int minimum_gain) {
  Gil gil;
  try {
    impl_->ensure();
    Object points(PyList_New(0));
    Object direction(ascent?PyFloat_FromDouble(*ascent):Py_NewRef(Py_None));
    Object band(floor?Py_BuildValue("(dd)",(*floor)[0],(*floor)[1]):Py_NewRef(Py_None));
    Object stances(stance?Py_BuildValue("[((ddd)iO)]",(*stance)[0],(*stance)[1],(*stance)[2],0,Py_True):
                          Py_NewRef(Py_None));
    if(!points.p || !direction.p || !band.p || !stances.p) throw std::runtime_error(python_error());
    for(const auto &target:targets) {
      Object p(Py_BuildValue("(ddd)",target[0],target[1],target[2]));
      if(!p.p || PyList_Append(points.p,p.p)<0) throw std::runtime_error(python_error());
    }
    Object method(PyObject_GetAttrString(impl_->backend,"observation_poses"));
    Object args(Py_BuildValue("((ddd)(ddd)O)",base[0],base[1],base[2],
        attitude[0],attitude[1],attitude[2],points.p));
    Object kwargs(Py_BuildValue("{s:K,s:O,s:O,s:O,s:O,s:d,s:i}","version",static_cast<unsigned long long>(version),
        "motions",include_moves?(*include_moves?Py_True:Py_False):Py_None,"ascent",direction.p,"floor",band.p,
        "stances",stances.p,"tolerance",tolerance,"minimum_gain",minimum_gain));
    Object answer(method.p && args.p && kwargs.p?PyObject_Call(method.p,args.p,kwargs.p):nullptr);
    if(!answer.p)throw std::runtime_error(python_error());
    auto *values=PyDict_GetItemString(answer.p,"poses");
    if(!values || !PyList_Check(values))throw std::runtime_error("invalid observation poses");
    Object rows(Py_NewRef(values));
    std::vector<PctObservationPose> out;
    for(Py_ssize_t i=0;i<PyList_Size(rows.p);++i) {
      auto *row=PyList_GetItem(rows.p,i);
      if(!PyList_Check(row) || PyList_Size(row)!=8)throw std::runtime_error("invalid observation pose");
      std::array<double,8> v;
      for(std::size_t j=0;j<v.size();++j) {
        v[j]=PyFloat_AsDouble(PyList_GetItem(row,j));
        if(PyErr_Occurred() || !std::isfinite(v[j]))throw std::runtime_error("invalid observation pose");
      }
      out.push_back({{v[0],v[1],v[2]},{v[3],v[4],v[5]},v[6],static_cast<PctObservationPose::Motion>(v[7])});
    }
    impl_->error.clear();
    return out;
  } catch(const std::exception &e){PyErr_Clear();impl_->error=e.what();return {};}
}
PctResult PctBackend::plan_with_method(const char *method,std::uint64_t version,const Point &start,
    const Point &target,std::optional<XY> floor) {
  PctResult out;
  Gil gil;
  try {
    Object answer(impl_->call(method,version,start,&target,floor));
    if(!PyTuple_Check(answer.p) || PyTuple_Size(answer.p)!=3) throw std::runtime_error("invalid PCT result");
    const char *reason=PyUnicode_AsUTF8(PyTuple_GetItem(answer.p,0));
    if(!reason) throw std::runtime_error(python_error());
    out.reason=reason;
    PyObject *rows=PyTuple_GetItem(answer.p,1);
    if(!PyList_Check(rows)) throw std::runtime_error("invalid PCT route");
    for(Py_ssize_t i=0;i<PyList_Size(rows);++i) {
      PyObject *row=PyList_GetItem(rows,i);
      if(!PyList_Check(row) || PyList_Size(row)!=3) throw std::runtime_error("invalid PCT XYZ");
      Point point{};
      for(int j=0;j<3;++j) point[j]=PyFloat_AsDouble(PyList_GetItem(row,j));
      if(PyErr_Occurred()) throw std::runtime_error(python_error());
      if(!std::isfinite(point[0])||!std::isfinite(point[1])||!std::isfinite(point[2])) throw std::runtime_error("invalid PCT XYZ");
      out.points.push_back(point);
    }
    PyObject *details=PyTuple_GetItem(answer.p,2);
    if(PyDict_Check(details)) {
      out.unknown_blockers=read_unknown_blockers(details,version);
      for(const auto &entry:{std::pair<const char*,std::optional<bool>*>{"start_check",&out.start_safe},
                             {"goal_check",&out.goal_safe}}) {
        PyObject *check=PyDict_GetItemString(details,entry.first);
        PyObject *ready=check && PyDict_Check(check)?PyDict_GetItemString(check,"ready"):nullptr;
        if(ready && PyBool_Check(ready)) *entry.second=(ready==Py_True);
      }
      if(auto *check=PyDict_GetItemString(details,"start_check");check && PyDict_Check(check)) {
        auto *height=PyDict_GetItemString(check,"ground_z");
        if(out.start_safe.value_or(false) && height && height!=Py_None) {
          const auto z=PyFloat_AsDouble(height);
          if(PyErr_Occurred() || !std::isfinite(z))throw std::runtime_error("invalid PCT start height");
          out.start_ground_z=z;
        }
      }
    }
    Object json(PyImport_ImportModule("json"));
    Object metadata(json.p?PyObject_CallMethod(json.p,"dumps","O",PyTuple_GetItem(answer.p,2)):nullptr);
    if(!metadata.p) throw std::runtime_error(python_error());
    out.metadata=PyUnicode_AsUTF8(metadata.p);
    impl_->error.clear();
  }catch(const std::exception &e){
    impl_->error=e.what();out.reason="pct_backend_unavailable";out.points.clear();
    PyErr_Clear();
    Object json(PyImport_ImportModule("json"));
    Object message(PyUnicode_FromString(impl_->error.c_str()));
    Object encoded(json.p && message.p?PyObject_CallMethod(json.p,"dumps","O",message.p):nullptr);
    if(encoded.p) out.metadata=std::string("{\"backend\":\"project_pct\",\"error\":")+
        PyUnicode_AsUTF8(encoded.p)+"}";
    else PyErr_Clear();
  }
  return out;
}
PctFrontiers PctBackend::frontiers(std::uint64_t version,const Point &base,double minimum_distance,
    std::optional<Point> window,double query_budget,std::optional<Point> continuation,
    std::optional<XY> floor,bool geometry_only,std::optional<XY> platform_floor,
    std::optional<FloorViewLimits> floor_selection,
    std::optional<double> stair_heading) {
  PctFrontiers out;
  Gil gil;
  try {
    impl_->ensure();
    Object buffer(PyBytes_FromStringAndSize("",0));
    Object bounds(window?Py_BuildValue("(ddd)",(*window)[0],(*window)[1],(*window)[2]):Py_NewRef(Py_None));
    Object forward(continuation?Py_BuildValue("(ddd)",(*continuation)[0],(*continuation)[1],(*continuation)[2]):Py_NewRef(Py_None));
    Object floor_scope(floor?Py_BuildValue("(dd)",(*floor)[0],(*floor)[1]):Py_NewRef(Py_None));
    Object platform(platform_floor?Py_BuildValue("(dd)",(*platform_floor)[0],(*platform_floor)[1]):Py_NewRef(Py_None));
    Object selection(floor_selection?Py_BuildValue("(ddddidd)",floor_selection->xy_tolerance,
        floor_selection->first_budget,floor_selection->review_budget,floor_selection->review_slice,
        floor_selection->max_attempts,floor_selection->max_step,floor_selection->min_rise):Py_NewRef(Py_None));
    Object heading(stair_heading?PyFloat_FromDouble(*stair_heading):Py_NewRef(Py_None));
    Object answer(PyObject_CallMethod(impl_->backend,"frontiers","OK(ddd)dOdOOOOOO",buffer.p,
        static_cast<unsigned long long>(version),base[0],base[1],base[2],minimum_distance,
        bounds.p,query_budget,forward.p,floor_scope.p,geometry_only?Py_True:Py_False,platform.p,selection.p,heading.p));
    if(!answer.p) throw std::runtime_error(python_error());
    const auto field=[](PyObject *dict,const char *key)->PyObject* {
      auto *value=PyDict_Check(dict)?PyDict_GetItemString(dict,key):nullptr;
      if(!value) throw std::runtime_error("invalid PCT frontier field");
      return value;
    };
    const auto count=[&](PyObject *dict,const char *key) {
      auto value=PyLong_AsUnsignedLongLong(field(dict,key));
      if(PyErr_Occurred()) throw std::runtime_error(python_error());
      return value;
    };
    const auto point=[](PyObject *value) {
      if(!PyList_Check(value) || PyList_Size(value)!=3)
        throw std::runtime_error("invalid PCT frontier XYZ");
      Point p{};
      for(int j=0;j<3;++j) {
        p[j]=PyFloat_AsDouble(PyList_GetItem(value,j));
        if(PyErr_Occurred() || !std::isfinite(p[j]))
          throw std::runtime_error("invalid PCT frontier XYZ");
      }
      return p;
    };
    out.extraction_ms=PyFloat_AsDouble(field(answer.p,"extraction_ms"));
    out.version=count(answer.p,"version");
    out.frontier_cells=count(answer.p,"frontier_cells");
    out.known_surface_cells=count(answer.p,"known_surface_cells");
    out.safe_cells=count(answer.p,"safe_cells");
    out.no_observation_clusters=count(answer.p,"no_observation_clusters");
    if(auto *v=PyDict_GetItemString(answer.p,"unreachable_observers"))out.unreachable_observers=PyLong_AsSize_t(v);
    if(auto *v=PyDict_GetItemString(answer.p,"query_ms"))out.query_ms=PyFloat_AsDouble(v);
    if(auto *v=PyDict_GetItemString(answer.p,"query_pending"))out.query_pending=PyObject_IsTrue(v);
    if(auto *v=PyDict_GetItemString(answer.p,"query_checked_clusters"))out.query_checked_clusters=PyLong_AsSize_t(v);
    if(auto *v=PyDict_GetItemString(answer.p,"observer_prepare_ms"))out.observer_prepare_ms=PyFloat_AsDouble(v);
    if(auto *phases=PyDict_GetItemString(answer.p,"observer_phase_ms")) {
      if(!PyDict_Check(phases))throw std::runtime_error("invalid observer phase timing");
      PyObject *key,*value;Py_ssize_t at=0;
      while(PyDict_Next(phases,&at,&key,&value))out.observer_phase_ms[PyUnicode_AsUTF8(key)]=PyFloat_AsDouble(value);
    }
    if(auto *v=PyDict_GetItemString(answer.p,"pending_stair_candidates"))out.pending_stair_candidates=PyLong_AsSize_t(v);
    if(auto *v=PyDict_GetItemString(answer.p,"observer_geometry_ms"))out.observer_geometry_ms=PyFloat_AsDouble(v);
    if(auto *v=PyDict_GetItemString(answer.p,"observer_visibility_ms"))out.observer_visibility_ms=PyFloat_AsDouble(v);
    if(auto *v=PyDict_GetItemString(answer.p,"scope"))out.scope=PyUnicode_AsUTF8(v);
    if(auto *v=PyDict_GetItemString(answer.p,"scoped_frontier_cells"))out.scoped_frontier_cells=PyLong_AsSize_t(v);
    if(auto *v=PyDict_GetItemString(answer.p,"deferred_floor_frontier_cells"))out.deferred_floor_frontier_cells=PyLong_AsSize_t(v);
    if(auto *v=PyDict_GetItemString(answer.p,"disconnected_floor_frontier_cells"))out.disconnected_floor_frontier_cells=PyLong_AsSize_t(v);
    if(auto *v=PyDict_GetItemString(answer.p,"pending_review_clusters"))out.pending_review_clusters=PyLong_AsSize_t(v);
    if(auto *v=PyDict_GetItemString(answer.p,"reviewing"))out.reviewing=PyObject_IsTrue(v);
    if(auto *v=PyDict_GetItemString(answer.p,"fast_floor"))out.fast_floor=PyObject_IsTrue(v);
    if(auto *v=PyDict_GetItemString(answer.p,"fast_skipped_clusters"))out.fast_skipped_clusters=PyLong_AsSize_t(v);
    if(auto *v=PyDict_GetItemString(answer.p,"fast_skipped_cells"))out.fast_skipped_cells=PyLong_AsSize_t(v);
    for(auto entry:{std::make_pair("no_view_clusters",&out.no_view_clusters),
                    std::make_pair("timed_out_clusters",&out.observer_timeout_clusters)}) {
      auto *rows=PyDict_GetItemString(answer.p,entry.first);
      if(!rows)continue;
      if(!PyList_Check(rows))throw std::runtime_error("invalid PCT clusters without observers");
      for(Py_ssize_t i=0;i<PyList_Size(rows);++i)
        entry.second->push_back(PyLong_AsUnsignedLongLong(PyList_GetItem(rows,i)));
    }
    if(auto *v=PyDict_GetItemString(answer.p,"stair_flights"))out.stair_flights=PyLong_AsSize_t(v);
    if(auto *v=PyDict_GetItemString(answer.p,"stair_step_unsafe"))out.stair_step_unsafe=PyLong_AsSize_t(v);
    if(auto *v=PyDict_GetItemString(answer.p,"stair_index_ms"))out.stair_index_ms=PyFloat_AsDouble(v);
    if(auto *v=PyDict_GetItemString(answer.p,"stair_platform_visible"))out.stair_platform_visible=PyObject_IsTrue(v);
    if(auto *map=PyDict_GetItemString(answer.p,"map");map && PyDict_Check(map))
      if(auto *revision=PyDict_GetItemString(map,"geometry_version"))
        out.geometry_version=PyLong_AsUnsignedLongLong(revision);
    out.unknown_blockers=read_unknown_blockers(answer.p,out.version);
    for(auto entry:{std::make_pair("stair_targets",&out.stair_targets),
                    std::make_pair("stair_step_targets",&out.stair_step_targets),
                    std::make_pair("saved_stair_entrances",&out.saved_stair_entrances),
                    std::make_pair("stair_landing_targets",&out.stair_landing_targets)}) {
      auto *rows=PyDict_GetItemString(answer.p,entry.first);
      if(!rows) continue;
      if(!PyList_Check(rows)) throw std::runtime_error("invalid PCT stair views");
      for(Py_ssize_t i=0;i<PyList_Size(rows);++i) {
        auto *row=PyList_GetItem(rows,i);
        if(!PyList_Check(row) || PyList_Size(row)<4 || PyList_Size(row)>6)
          throw std::runtime_error("invalid PCT stair view");
        std::array<double,4> values;
        for(std::size_t j=0;j<values.size();++j) {
          values[j]=PyFloat_AsDouble(PyList_GetItem(row,j));
          if(PyErr_Occurred() || !std::isfinite(values[j])) throw std::runtime_error("invalid PCT stair view");
        }
        const auto evidence=PyList_Size(row)>=5?PyLong_AsUnsignedLongLong(PyList_GetItem(row,4)):0;
        if(PyErr_Occurred())throw std::runtime_error("invalid PCT stair evidence");
        entry.second->push_back({{values[0],values[1],values[2]},values[3],evidence});
      }
    }
    if(auto *v=PyDict_GetItemString(answer.p,"window_center");v && PyList_Check(v) && PyList_Size(v)==3)
      out.window_center=point(v);
    auto *clusters=field(answer.p,"clusters");
    if(!PyList_Check(clusters)) throw std::runtime_error("invalid PCT frontier clusters");
    for(Py_ssize_t i=0;i<PyList_Size(clusters);++i) {
      auto *item=PyList_GetItem(clusters,i);
      PctFrontier c;c.id=count(item,"id");c.revision=count(item,"revision");c.cells=count(item,"cells");
      c.frontier=point(field(item,"frontier"));
      c.gain=PyFloat_AsDouble(field(item,"gain"));
      if(auto *v=PyDict_GetItemString(item,"observer_ms"))c.observer_ms=PyFloat_AsDouble(v);
      if(auto *v=PyDict_GetItemString(item,"attempt"))c.attempt=PyLong_AsLong(v);
      if(auto *v=PyDict_GetItemString(item,"floor_failure"))c.floor_failure=PyUnicode_AsUTF8(v);
      if(auto *v=PyDict_GetItemString(item,"first_rejected"))c.first_rejected=point(v);
      c.unknown_blockers=read_unknown_blockers(item,version);
      auto *cell=field(item,"cell");
      if(!PyList_Check(cell) || PyList_Size(cell)!=2) throw std::runtime_error("invalid PCT cell");
      c.cell={static_cast<int>(PyLong_AsLong(PyList_GetItem(cell,0))),
              static_cast<int>(PyLong_AsLong(PyList_GetItem(cell,1)))};
      for(auto entry:{std::pair<const char*,std::vector<Point>*>{"observations",&c.observations},
                      {"display",&c.display}}) {
        auto *rows=field(item,entry.first);
        if(!PyList_Check(rows)) throw std::runtime_error("invalid PCT points");
        for(Py_ssize_t j=0;j<PyList_Size(rows);++j) entry.second->push_back(point(PyList_GetItem(rows,j)));
      }
      if(PyErr_Occurred() || !std::isfinite(c.gain)) throw std::runtime_error(python_error());
      if(auto *yaws=PyDict_GetItemString(item,"observation_yaws")) {
        if(!PyList_Check(yaws) || PyList_Size(yaws)!=static_cast<Py_ssize_t>(c.observations.size()))
          throw std::runtime_error("invalid observation headings");
        for(Py_ssize_t j=0;j<PyList_Size(yaws);++j) {
          const auto yaw=PyFloat_AsDouble(PyList_GetItem(yaws,j));
          if(!std::isfinite(yaw))throw std::runtime_error("nonfinite observation heading");
          c.observation_yaws.push_back(yaw);
        }
      }
      out.clusters.push_back(std::move(c));
    }
    if(auto *metrics=PyDict_GetItemString(answer.p,"map")) {
      Object json(PyImport_ImportModule("json"));
      Object encoded(json.p?PyObject_CallMethod(json.p,"dumps","O",metrics):nullptr);
      if(!encoded.p) throw std::runtime_error(python_error());
      out.metadata=PyUnicode_AsUTF8(encoded.p);
    }
    out.valid=true;impl_->error.clear();
  } catch(const std::exception &e) {
    PyErr_Clear();impl_->error=e.what();out.error=e.what();out.clusters.clear();
  }
  return out;
}
PctStairProbe PctBackend::floor_stair_probe(
    std::uint64_t version,const Point &base,const XY &floor,double yaw,bool depart,bool review,
    double tolerance) {
  PctStairProbe out;
  Gil gil;
  try {
    impl_->ensure();
    Object buffer(PyBytes_FromStringAndSize("",0));
    Object answer(PyObject_CallMethod(impl_->backend,"floor_stair_probe","OK(ddd)(dd)dOOd",buffer.p,
        static_cast<unsigned long long>(version),base[0],base[1],base[2],floor[0],floor[1],yaw,
        depart?Py_True:Py_False,review?Py_True:Py_False,tolerance));
    if(!answer.p)throw std::runtime_error(python_error());
    auto *ready=PyDict_GetItemString(answer.p,"ready");
    out.active=PyDict_GetItemString(answer.p,"active")==Py_True;
    out.finished=PyDict_GetItemString(answer.p,"finished")==Py_True;
    out.returning=PyDict_GetItemString(answer.p,"returning")==Py_True;
    out.approaching=PyDict_GetItemString(answer.p,"approaching")==Py_True;
    out.climbing=PyDict_GetItemString(answer.p,"climbing")==Py_True;
    if(auto *reason=PyDict_GetItemString(answer.p,"reason"))out.reason=PyUnicode_AsUTF8(reason);
    Object json(PyImport_ImportModule("json"));
    Object encoded(json.p?PyObject_CallMethod(json.p,"dumps","O",answer.p):nullptr);
    if(!encoded.p)throw std::runtime_error(python_error());
    out.metadata=PyUnicode_AsUTF8(encoded.p);
    auto *points=PyDict_GetItemString(answer.p,"observations");
    auto *headings=PyDict_GetItemString(answer.p,"yaws");
    if(ready==Py_True && points && headings && PyList_Check(points) && PyList_Check(headings) &&
       PyList_Size(points)==PyList_Size(headings)) {
      out.frontier.revision=version;
      for(Py_ssize_t i=0;i<PyList_Size(points);++i) {
        auto *row=PyList_GetItem(points,i);
        if(!PySequence_Check(row) || PySequence_Size(row)!=3)throw std::runtime_error("invalid PCT stair pose");
        Point p;
        for(int j=0;j<3;++j) {Object v(PySequence_GetItem(row,j));p[j]=PyFloat_AsDouble(v.p);}
        const double heading=PyFloat_AsDouble(PyList_GetItem(headings,i));
        if(PyErr_Occurred() || !std::isfinite(heading) ||
           !std::all_of(p.begin(),p.end(),[](double v){return std::isfinite(v);}))
          throw std::runtime_error("invalid PCT stair pose");
        out.frontier.observations.push_back(p);out.frontier.observation_yaws.push_back(heading);
      }
      if(!out.frontier.observations.empty())out.frontier.frontier=out.frontier.observations.front();
    }
    impl_->error.clear();
  } catch(const std::exception &e) {PyErr_Clear();impl_->error=e.what();}
  return out;
}

bool PctBackend::complete_observation(std::uint64_t version,const PctFrontier &frontier,const Point &point) {
  Gil gil;
  try {
    impl_->ensure();Object buffer(PyBytes_FromStringAndSize("",0));
    Object answer(PyObject_CallMethod(impl_->backend,"complete_observation","OKKK(ddd)",buffer.p,
        static_cast<unsigned long long>(version),static_cast<unsigned long long>(frontier.id),
        static_cast<unsigned long long>(frontier.revision),point[0],point[1],point[2]));
    if(!answer.p) throw std::runtime_error(python_error());
    impl_->error.clear();return true;
  } catch(const std::exception &e) {PyErr_Clear();impl_->error=e.what();return false;}
}
PctObservation PctBackend::observation_task(std::uint64_t version,const std::string &operation,
    std::uint64_t cluster,const Point &point,double yaw,double half_angle,
    const std::vector<PctUnknownBlocker> *blockers) {
  PctObservation out;Gil gil;
  try {
    impl_->ensure();Object buffer(PyBytes_FromStringAndSize("",0));
    Object gaps(blockers?PyList_New(0):Py_NewRef(Py_None));
    if(!gaps.p)throw std::runtime_error(python_error());
    if(blockers)for(const auto &entry:*blockers) {
      Object gap(Py_BuildValue("[dddi]",entry.point[0],entry.point[1],entry.point[2],entry.slice));
      if(!gap.p || PyList_Append(gaps.p,gap.p)<0)throw std::runtime_error(python_error());
    }
    Object answer(PyObject_CallMethod(impl_->backend,"observation_task","OKsK(ddd)ddO",buffer.p,
        static_cast<unsigned long long>(version),operation.c_str(),static_cast<unsigned long long>(cluster),
        point[0],point[1],point[2],yaw,half_angle,gaps.p));
    if(!answer.p || !PyDict_Check(answer.p))throw std::runtime_error(python_error());
    if(auto *v=PyDict_GetItemString(answer.p,"ready"))out.ready=PyObject_IsTrue(v);
    if(auto *v=PyDict_GetItemString(answer.p,"fresh"))out.fresh=PyObject_IsTrue(v);
    if(auto *v=PyDict_GetItemString(answer.p,"yaw"))out.yaw=PyFloat_AsDouble(v);
    if(auto *v=PyDict_GetItemString(answer.p,"side_yaw");v && v!=Py_None)out.side_yaw=PyFloat_AsDouble(v);
    if(auto *v=PyDict_GetItemString(answer.p,"gain"))out.gain=PyLong_AsSize_t(v);
    if(auto *v=PyDict_GetItemString(answer.p,"region");v && PyList_Check(v) && PyList_Size(v)==3) {
      out.region=Point{};
      for(int i=0;i<3;++i)(*out.region)[i]=PyFloat_AsDouble(PyList_GetItem(v,i));
    }
    out.unknown_blockers=read_unknown_blockers(answer.p,version);
    if(auto *v=PyDict_GetItemString(answer.p,"floor_attempt");v && PyUnicode_Check(v))out.floor_attempt=PyUnicode_AsUTF8(v);
    if(auto *v=PyDict_GetItemString(answer.p,"reason"))out.reason=PyUnicode_AsUTF8(v);
    if(PyErr_Occurred())throw std::runtime_error(python_error());
    Object json(PyImport_ImportModule("json"));Object encoded(PyObject_CallMethod(json.p,"dumps","O",answer.p));
    if(!encoded.p)throw std::runtime_error(python_error());
    out.metadata=PyUnicode_AsUTF8(encoded.p);impl_->error.clear();
  }catch(const std::exception &e){PyErr_Clear();impl_->error=e.what();out.reason=e.what();}
  return out;
}
PctObservation PctBackend::arrival_area(std::uint64_t version,const Point &target,double xy_tolerance,double heading) {
  return observation_task(version,"arrival_area",0,target,heading,xy_tolerance);
}
std::vector<Point> PctBackend::recovery_targets(std::uint64_t version,const Point &start) {
  return read_points("recovery_targets",version,start);
}

std::vector<Point> PctBackend::stair_review_points(std::uint64_t version,const Point &base,
    double heading) {
  return read_points("stair_review_points",version,base,heading);
}

std::vector<Point> PctBackend::read_points(const char *method,std::uint64_t version,
    const Point &base,std::optional<double> heading) {
  std::vector<Point> out;Gil gil;
  try {
    Object answer(impl_->call(method,version,base,nullptr,std::nullopt,heading));
    if(!PyList_Check(answer.p)) throw std::runtime_error("invalid PCT recovery targets");
    for(Py_ssize_t i=0;i<PyList_Size(answer.p);++i) {
      auto *row=PyList_GetItem(answer.p,i);
      if(!PyList_Check(row) || PyList_Size(row)!=3) throw std::runtime_error("invalid recovery XYZ");
      Point p{};
      for(int j=0;j<3;++j) p[j]=PyFloat_AsDouble(PyList_GetItem(row,j));
      if(PyErr_Occurred() || !std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]))
        throw std::runtime_error("invalid recovery XYZ");
      out.push_back(p);
    }
    impl_->error.clear();
  }catch(const std::exception &e){PyErr_Clear();impl_->error=e.what();}
  return out;
}
PctPointCheck PctBackend::point_check(std::uint64_t version,const Point &point,bool platform,
                                    std::optional<XY> departure_floor,double landing_heading,bool landing_recovery,
                                    std::optional<XY> floor,bool tread_support,std::optional<double> platform_reference) {
  PctPointCheck out;Gil gil;
  try {
    const auto method=departure_floor?(landing_recovery?"landing_recovery_check":"landing_check"):
        platform?"platform_check":tread_support?"tread_support_check":"point_check";
    Object answer(impl_->call(method,version,point,nullptr,departure_floor?departure_floor:floor,
                              departure_floor?std::optional<double>(landing_heading):std::nullopt,
                              platform_reference));
    auto *ready=PyDict_GetItemString(answer.p,"ready");
    auto *reason=PyDict_GetItemString(answer.p,"reason");
    if(!ready || !PyBool_Check(ready) || !reason) throw std::runtime_error("invalid PCT point check");
    out.ready=ready==Py_True;out.reason=PyUnicode_AsUTF8(reason);
    const auto read_height=[&](const char *key)->std::optional<double> {
      auto *height=PyDict_GetItemString(answer.p,key);
      if(!height || height==Py_None)return std::nullopt;
      const double z=PyFloat_AsDouble(height);
      if(PyErr_Occurred() || !std::isfinite(z)) throw std::runtime_error("invalid PCT height");
      return z;
    };
    out.ground_z=read_height("ground_z");
    out.platform_height=read_height("platform_height");
    auto *physical=PyDict_GetItemString(answer.p,"physical_footprint_conflict");
    out.physical_conflict=physical && physical==Py_True;
    Object json(PyImport_ImportModule("json"));
    Object encoded(json.p?PyObject_CallMethod(json.p,"dumps","O",answer.p):nullptr);
    if(!encoded.p) throw std::runtime_error(python_error());
    out.metadata=PyUnicode_AsUTF8(encoded.p);impl_->error.clear();
  }catch(const std::exception &e){PyErr_Clear();impl_->error=e.what();out.reason="pct_backend_unavailable";}
  return out;
}
bool PctBackend::start_ready(std::uint64_t version,const Point &start,std::optional<XY> floor) {
  Gil gil;
  try {
    Object answer(impl_->call("start_ready",version,start,nullptr,floor));
    const int ready=PyObject_IsTrue(answer.p);
    if(ready<0) throw std::runtime_error(python_error());
    impl_->error.clear();return ready>0;
  }catch(const std::exception &e){impl_->error=e.what();return false;}
}
std::optional<std::uint64_t> PctBackend::recovery_evidence(
    std::uint64_t version,const Point &base,std::optional<Point> forward) {
  Gil gil;
  try {
    Object answer(impl_->call("recovery_evidence",version,base,forward?&*forward:nullptr));
    const auto signature=PyLong_AsUnsignedLongLong(answer.p);
    if(PyErr_Occurred())throw std::runtime_error(python_error());
    impl_->error.clear();return signature;
  } catch(const std::exception &e){PyErr_Clear();impl_->error=e.what();return std::nullopt;}
}
PctInspection PctBackend::inspect(std::uint64_t version,const Point &base,bool display,
    std::size_t max_display,bool preview,double radius,double min_gain,
    std::size_t max_candidates,std::size_t max_nodes) {
  PctInspection out;Gil gil;
  try {
    impl_->ensure();
    Object answer(PyObject_CallMethod(impl_->backend,"inspect","K(ddd)(iKiddKK)",
        static_cast<unsigned long long>(version),base[0],base[1],base[2],display?1:0,
        static_cast<unsigned long long>(max_display),preview?1:0,radius,min_gain,
        static_cast<unsigned long long>(max_candidates),static_cast<unsigned long long>(max_nodes)));
    if(!answer.p || !PyDict_Check(answer.p)) throw std::runtime_error(python_error());
    const auto bytes=[&](const char *key,std::vector<std::uint8_t> &target) {
      PyObject *p=PyDict_GetItemString(answer.p,key);
      if(!p || !PyBytes_Check(p)) throw std::runtime_error("invalid PCT cloud payload");
      const auto *first=reinterpret_cast<std::uint8_t*>(PyBytes_AsString(p));
      target.assign(first,first+PyBytes_Size(p));
    };
    bytes("surfaces",out.surfaces);bytes("display",out.display);
    bytes("returns",out.returns);bytes("ceilings",out.ceilings);
    const auto points=[&](const char *key,std::vector<Point> &target) {
      PyObject *rows=PyDict_GetItemString(answer.p,key);
      if(!rows || !PyList_Check(rows)) throw std::runtime_error("invalid PCT preview payload");
      for(Py_ssize_t i=0;i<PyList_Size(rows);++i) {
        PyObject *row=PyList_GetItem(rows,i);
        if(!PyList_Check(row) || PyList_Size(row)!=3) throw std::runtime_error("invalid preview XYZ");
        Point p{};for(int j=0;j<3;++j)p[j]=PyFloat_AsDouble(PyList_GetItem(row,j));
        if(PyErr_Occurred())throw std::runtime_error(python_error());target.push_back(p);
      }
    };
    points("nodes",out.preview_nodes);points("candidates",out.preview_candidates);points("path",out.path);
    PyObject *state=PyDict_GetItemString(answer.p,"preview_state");
    if(!state || !PyUnicode_Check(state))throw std::runtime_error("invalid PCT preview state");
    out.preview_state=PyUnicode_AsUTF8(state);
    Object json(PyImport_ImportModule("json"));
    Object encoded(json.p?PyObject_CallMethod(json.p,"dumps","O",PyDict_GetItemString(answer.p,"metadata")):nullptr);
    if(!encoded.p)throw std::runtime_error(python_error());
    out.metadata=PyUnicode_AsUTF8(encoded.p);out.valid=true;impl_->error.clear();
  }catch(const std::exception &e){PyErr_Clear();impl_->error=e.what();out.preview_state="pct_inspection_unavailable";}
  return out;
}
std::size_t PctBackend::point_count() const{return impl_->surface_records;}
const std::string &PctBackend::last_error() const{return impl_->error;}
const std::string &PctBackend::map_metadata() const{return impl_->metadata;}
} // namespace legbot_exploration
