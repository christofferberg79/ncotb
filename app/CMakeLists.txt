add_executable(ncotb main.cpp app.cpp cannon.cpp draw.cpp interaction.cpp scenes.cpp track.cpp)
target_link_libraries(ncotb PRIVATE physics imgui raylib)
ncotb_warnings(ncotb)
if(APPLE)
  target_link_libraries(ncotb PRIVATE "-framework IOKit" "-framework Cocoa" "-framework OpenGL")
endif()

# Release packaging (the "package" preset): a double-clickable ncotb.app on macOS, a
# console-less ncotb.exe on Windows, installed for CPack (see the root CMakeLists.txt).
if(NCOTB_PACKAGE)
  if(APPLE)
    set_target_properties(ncotb PROPERTIES
      MACOSX_BUNDLE ON
      MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/Info.plist.in
      MACOSX_BUNDLE_BUNDLE_NAME ncotb
      MACOSX_BUNDLE_GUI_IDENTIFIER io.github.christofferberg79.ncotb
      MACOSX_BUNDLE_SHORT_VERSION_STRING ${NCOTB_VERSION}
      MACOSX_BUNDLE_BUNDLE_VERSION ${NCOTB_VERSION})
  elseif(WIN32)
    # mingw's startup code still calls main() for a GUI-subsystem executable.
    set_target_properties(ncotb PROPERTIES WIN32_EXECUTABLE ON)
  endif()
  install(TARGETS ncotb BUNDLE DESTINATION . RUNTIME DESTINATION .)
  if(APPLE)
    # Sign the staged bundle (ad hoc: no Apple developer ID), after anything install
    # rewrote. Without a valid signature a downloaded app is reported as "damaged";
    # with one, Gatekeeper only asks for a one-time "Open Anyway".
    install(CODE [[
      execute_process(COMMAND codesign --force --sign - "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/ncotb.app"
                      COMMAND_ERROR_IS_FATAL ANY)
    ]])
  endif()
endif()
