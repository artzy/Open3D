include(ExternalProject)

if(MSVC)
    set(lib_name zlibstatic)
else()
    set(lib_name z)
endif()

find_package(Git QUIET REQUIRED)

# On MSVC, building the patched zlib shared target (zlib1.dll) fails with LNK2005
# duplicate symbols. Open3D only links zlibstatic on Windows.
if(MSVC)
    set(ZLIB_EP_EXTRA_ARGS
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR> --config Release --target zlibstatic
        INSTALL_COMMAND
            ${CMAKE_COMMAND} -E make_directory <INSTALL_DIR>/include <INSTALL_DIR>/lib
            COMMAND ${CMAKE_COMMAND} -E copy
                <BINARY_DIR>/Release/${CMAKE_STATIC_LIBRARY_PREFIX}${lib_name}${CMAKE_STATIC_LIBRARY_SUFFIX}
                <INSTALL_DIR>/lib/${CMAKE_STATIC_LIBRARY_PREFIX}${lib_name}${CMAKE_STATIC_LIBRARY_SUFFIX}
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                <SOURCE_DIR>/zlib.h <INSTALL_DIR>/include/zlib.h
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                <BINARY_DIR>/zconf.h <INSTALL_DIR>/include/zconf.h
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                <SOURCE_DIR>/contrib/minizip/unzip.h <INSTALL_DIR>/include/unzip.h
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                <SOURCE_DIR>/contrib/minizip/ioapi.h <INSTALL_DIR>/include/ioapi.h
    )
else()
    set(ZLIB_EP_EXTRA_ARGS "")
endif()

ExternalProject_Add(
    ext_zlib
    PREFIX zlib
    URL https://github.com/madler/zlib/releases/download/v1.3.1/zlib-1.3.1.tar.xz
    URL_HASH SHA256=38ef96b8dfe510d42707d9c781877914792541133e1870841463bfa73f883e32
    DOWNLOAD_DIR "${OPEN3D_THIRD_PARTY_DOWNLOAD_DIR}/zlib"
    UPDATE_COMMAND ""
    PATCH_COMMAND ${GIT_EXECUTABLE} init
    COMMAND       ${GIT_EXECUTABLE} apply --ignore-space-change --ignore-whitespace
                  ${CMAKE_CURRENT_LIST_DIR}/0001-patch-zlib-to-enable-unzip.patch
    CMAKE_ARGS
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5
        -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
        # zlib needs visible symbols for examples. Disabling example building causes
        # assember error in GPU CI. zlib symbols are hidden during linking.
        ${ExternalProject_CMAKE_ARGS}
    ${ZLIB_EP_EXTRA_ARGS}
    BUILD_BYPRODUCTS
        <INSTALL_DIR>/lib/${CMAKE_STATIC_LIBRARY_PREFIX}${lib_name}${CMAKE_STATIC_LIBRARY_SUFFIX}
        <INSTALL_DIR>/lib/${CMAKE_STATIC_LIBRARY_PREFIX}${lib_name}d${CMAKE_STATIC_LIBRARY_SUFFIX}
)

ExternalProject_Get_Property(ext_zlib INSTALL_DIR)
set(ZLIB_INCLUDE_DIRS ${INSTALL_DIR}/include/) # "/" is critical.
set(ZLIB_LIB_DIR ${INSTALL_DIR}/lib)
if(MSVC)
    set(ZLIB_LIBRARIES ${lib_name}$<$<CONFIG:Debug>:d>)
else()
    set(ZLIB_LIBRARIES ${lib_name})
endif()
