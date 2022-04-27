if(NOT EXISTS "/home/isorrentino/dev/iCub-torque-control/build/install_manifest.txt")
    message(WARNING "Cannot find install manifest: \"/home/isorrentino/dev/iCub-torque-control/build/install_manifest.txt\"")
    return()
endif()
file(READ "/home/isorrentino/dev/iCub-torque-control/build/install_manifest.txt" files)
string(STRIP "" files)
string(REGEX REPLACE "\n" ";" files "")
list(REVERSE files)
foreach(file )
    message(STATUS "Uninstalling: ")
    if(EXISTS "")
        execute_process(
            COMMAND /usr/bin/cmake -E remove ""
            OUTPUT_VARIABLE rm_out
            RESULT_VARIABLE rm_retval)
        if(NOT "" EQUAL 0)
            message(FATAL_ERROR "Problem when removing \"\"")
        endif()
    else()
        message(STATUS "File \"\" does not exist.")
    endif()
endforeach(file)
