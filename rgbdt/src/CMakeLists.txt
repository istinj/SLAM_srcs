add_library(lk_tracker_library
  point_track.cpp
  frame_info.cpp
  key_frame.cpp
  landmark.cpp
  posit_solver.cpp
  lk_tracker.cpp
  frame_info_manager.cpp
  opengl_primitives.cpp
  simple_viewer.cpp
  lk_viewer.cpp
)

TARGET_LINK_LIBRARIES(lk_tracker_library
  ${G2O_OPENGL_HELPER_LIBRARY}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_TYPES_SLAM3D} 
  ${G2O_CORE_LIBRARY} 
  ${G2O_STUFF_LIBRARY} 
  ${CSPARSE_LIBRARY}
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${OpenCV_LIBS}
)

add_dependencies(lk_tracker_library ${catkin_EXPORTED_TARGETS})

add_executable(lk_app  lk_app.cpp )
TARGET_LINK_LIBRARIES(lk_app
  lk_tracker_library
  thin_txt_io_library
  thin_ros_wrappers_library
  ${G2O_OPENGL_HELPER_LIBRARY}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_TYPES_SLAM3D} 
  ${G2O_CORE_LIBRARY} 
  ${G2O_STUFF_LIBRARY} 
  ${CSPARSE_LIBRARY}
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${OpenCV_LIBS})

add_dependencies(lk_app ${catkin_EXPORTED_TARGETS})

# add_executable(lk_node lk_node.cpp lk_ros_bridge.cpp)
# TARGET_LINK_LIBRARIES(lk_node
#   lk_tracker_library
#   thin_txt_io_library
#   thin_ros_wrappers_library
#   gl_helpers_library
#   ${G2O_OPENGL_HELPER_LIBRARY}
#   ${G2O_SOLVER_CSPARSE_EXTENSION}
#   ${G2O_TYPES_SLAM3D} 
#   ${G2O_CORE_LIBRARY} 
#   ${G2O_STUFF_LIBRARY} 
#   ${CSPARSE_LIBRARY}
#   ${QGLVIEWER_LIBRARY} 
#   ${QT_QTXML_LIBRARY} 
#   ${QT_QTOPENGL_LIBRARY} 
#   ${QT_QTGUI_LIBRARY} 
#   ${QT_QTCORE_LIBRARY} 
#   ${OPENGL_gl_LIBRARY} 
#   ${OPENGL_glu_LIBRARY}
#   ${OpenCV_LIBS}  
#   ${catkin_LIBRARIES})

# add_dependencies(lk_node ${catkin_EXPORTED_TARGETS})

# add_executable(lk_viewer_node lk_viewer_node.cpp lk_ros_bridge.cpp)
# TARGET_LINK_LIBRARIES(lk_viewer_node
#   lk_tracker_library
#   thin_txt_io_library
#   thin_ros_wrappers_library
#   gl_helpers_library
#   ${QGLVIEWER_LIBRARY} 
#   ${QT_QTXML_LIBRARY} 
#   ${QT_QTOPENGL_LIBRARY} 
#   ${QT_QTGUI_LIBRARY} 
#   ${QT_QTCORE_LIBRARY} 
#   ${OPENGL_gl_LIBRARY} 
#   ${OPENGL_glu_LIBRARY}
#   ${OpenCV_LIBS}  
#   ${catkin_LIBRARIES})

# add_dependencies(lk_viewer_node ${catkin_EXPORTED_TARGETS})
