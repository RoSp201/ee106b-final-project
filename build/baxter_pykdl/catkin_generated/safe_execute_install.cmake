execute_process(COMMAND "/home/cc/ee106b/sp17/class/ee106b-aah/final_project/build/baxter_pykdl/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/cc/ee106b/sp17/class/ee106b-aah/final_project/build/baxter_pykdl/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
