

set(command "/usr/bin/cmake;-P;/home/teja/MeshSegmentation/ThirdParty/tmp/googletest-gitclone.cmake")
execute_process(
  COMMAND ${command}
  RESULT_VARIABLE result
  OUTPUT_FILE "/home/teja/MeshSegmentation/ThirdParty/src/googletest-stamp/googletest-download-out.log"
  ERROR_FILE "/home/teja/MeshSegmentation/ThirdParty/src/googletest-stamp/googletest-download-err.log"
  )
if(result)
  set(msg "Command failed: ${result}\n")
  foreach(arg IN LISTS command)
    set(msg "${msg} '${arg}'")
  endforeach()
  set(msg "${msg}\nSee also\n  /home/teja/MeshSegmentation/ThirdParty/src/googletest-stamp/googletest-download-*.log")
  message(FATAL_ERROR "${msg}")
else()
  set(msg "googletest download command succeeded.  See also /home/teja/MeshSegmentation/ThirdParty/src/googletest-stamp/googletest-download-*.log")
  message(STATUS "${msg}")
endif()
