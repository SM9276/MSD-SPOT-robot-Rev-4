# generated from
# ament_cmake_core/cmake/symlink_install/ament_cmake_symlink_install_uninstall_script.cmake.in

<<<<<<< HEAD
<<<<<<< HEAD
set(install_manifest "/mnt/c/Users/Oden2/Downloads/MSD-SPOT-robot-Rev-4/build/gear_manipulator_moveit_config/symlink_install_manifest.txt")
=======
set(install_manifest "/home/msd/MSD2/REV4/build/gear_manipulator_moveit_config/symlink_install_manifest.txt")
>>>>>>> f6515614f97110be7e36a5fdeab81499e9790a26
=======
set(install_manifest "/home/msd/MSD2/REV4/build/gear_manipulator_moveit_config/symlink_install_manifest.txt")
>>>>>>> f6515614f97110be7e36a5fdeab81499e9790a26
if(NOT EXISTS "${install_manifest}")
  message(FATAL_ERROR "Cannot find symlink install manifest: ${install_manifest}")
endif()

file(READ "${install_manifest}" installed_files)
string(REGEX REPLACE "\n" ";" installed_files "${installed_files}")
foreach(installed_file ${installed_files})
  if(EXISTS "${installed_file}" OR IS_SYMLINK "${installed_file}")
    message(STATUS "Uninstalling: ${installed_file}")
    file(REMOVE "${installed_file}")
    if(EXISTS "${installed_file}" OR IS_SYMLINK "${installed_file}")
      message(FATAL_ERROR "Failed to remove '${installed_file}'")
    endif()

    # remove empty parent folders
    get_filename_component(parent_path "${installed_file}" PATH)
    ament_cmake_uninstall_target_remove_empty_directories("${parent_path}")
  endif()
endforeach()
