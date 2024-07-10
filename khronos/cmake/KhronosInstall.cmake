install(
  TARGETS ${PROJECT_NAME}
  EXPORT khronos-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(
  EXPORT khronos-targets
  FILE khronosTargets.cmake
  NAMESPACE khronos::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/khronos
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/khronosConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/khronosConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/khronos
)
write_basic_package_version_file(
  khronosConfigVersion.cmake
  VERSION ${PACKAGE_VERSION}
  COMPATIBILITY AnyNewerVersion
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/khronosConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/khronosConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/khronos
)
