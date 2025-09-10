CMAKE_MINIMUM_REQUIRED (VERSION 3.16)
PROJECT                (tests)

SET (ProjectName  Project)
SET (TestsName    Tests)
SET (LibraryName  Lib)

SET (PackageNames i2c rtos utils logger loggerhw)
SET (PackageIncludePath                        /home/sylwester/.conan2/download/packages)
SET (CurrentPath                               ${CMAKE_CURRENT_SOURCE_DIR})

SET  (CommonIncludePath                        ${CurrentPath}/../Common)
FILE (GLOB_RECURSE CommonFilesAndFolders       ${CommonIncludePath}/*)

SET  (ProjectIncludePath                       ${CurrentPath}/../${ProjectName})
FILE (GLOB_RECURSE ProjectFilesAndFolders      ${ProjectIncludePath}/*)

SET  (TestIncludePath                          ${CurrentPath})
FILE (GLOB_RECURSE TestFilesAndFolders         ${TestIncludePath}/*)

SET  (ProtoGeneratedPath                       ${CMAKE_BINARY_DIR}/Generated)
FILE (MAKE_DIRECTORY                           ${ProtoGeneratedPath})
FILE (GLOB_RECURSE ProtoFiles                  ${CommonIncludePath}/MsgType/*.proto)

FUNCTION (EXCLUDE_PATHS paths filter)
    FOREACH (path ${${paths}})
        if ("${path}" MATCHES ${filter})
            LIST (REMOVE_ITEM ${paths} ${path})
        endif ("${path}" MATCHES ${filter})
    ENDFOREACH (path)
    SET (${paths} ${${paths}} PARENT_SCOPE)
ENDFUNCTION (EXCLUDE_PATHS)

EXCLUDE_PATHS (CommonFilesAndFolders  /Hardware/*)
EXCLUDE_PATHS (ProjectFilesAndFolders /build/*)
EXCLUDE_PATHS (ProjectFilesAndFolders /.vscode/*)
EXCLUDE_PATHS (ProjectFilesAndFolders /main/Hardware/*)
EXCLUDE_PATHS (ProjectFilesAndFolders /main/Middleware/Communication/SystemEvents/CircularBuffer/*)
EXCLUDE_PATHS (TestFilesAndFolders    /build/*)
EXCLUDE_PATHS (TestFilesAndFolders    /Build/*)
EXCLUDE_PATHS (TestFilesAndFolders    /Conan/*)
EXCLUDE_PATHS (TestFilesAndFolders    /.vscode/*)

MACRO (PARSE_SUBDIRECTORIES_PATHS searchDirectory paths)
    SET (directoryPaths "")
    FOREACH (subdirectoryPath ${searchDirectory})
        GET_FILENAME_COMPONENT (directoryPath ${subdirectoryPath} PATH)
        SET (directoryPaths ${directoryPaths} ${directoryPath})
    ENDFOREACH ()

    LIST (REMOVE_DUPLICATES directoryPaths)
    SET  (${paths} ${directoryPaths})
ENDMACRO ()

PARSE_SUBDIRECTORIES_PATHS ("${CommonFilesAndFolders}"  CommonPathsLists)
PARSE_SUBDIRECTORIES_PATHS ("${ProjectFilesAndFolders}" ProjectPathsLists)
PARSE_SUBDIRECTORIES_PATHS ("${TestFilesAndFolders}"    TestPathsLists)

MACRO (GET_PACKAGE_PATHS path folderName result)
    FOREACH (packageName ${PackageNames})
        STRING (CONCAT newPath "${path}" "/${packageName}/${folderName}")
        LIST   (APPEND ${result} ${newPath})
    ENDFOREACH (packageName)
ENDMACRO()

GET_PACKAGE_PATHS ("${PackageIncludePath}" "lib"     PackageLibsPaths)
GET_PACKAGE_PATHS ("${PackageIncludePath}" "include" PackageIncludePaths)

FIND_PACKAGE (GTest REQUIRED)

MACRO (PARSE_FILE_NAMES files names)
    SET (fileNames "")
    FOREACH (file ${files})
        GET_FILENAME_COMPONENT (name ${file} NAME)
        SET (fileNames ${fileNames} ${name})
    ENDFOREACH ()

    SET (${names} ${fileNames})
ENDMACRO ()

PARSE_FILE_NAMES ("${ProtoFiles}" ProtoFileNames)
EXECUTE_PROCESS  (COMMAND protoc-c --proto_path=${CommonIncludePath}/MsgType
                                   --c_out=${ProtoGeneratedPath}
                                   ${ProtoFileNames}
                                   WORKING_DIRECTORY ${CurrentPath}
                 )
FILE (GLOB_RECURSE ProtoFiles ${ProtoGeneratedPath}/*.pb-c.c)

SOURCE_GROUP               (TREE                              ${ProjectIncludePath} PREFIX "" FILES ${ProjectFilesAndFolders})
LINK_DIRECTORIES           (${ProjectName}                    ${PackageLibsPaths})
ADD_EXECUTABLE             (${ProjectName}                    ${CommonFilesAndFolders} ${ProjectFilesAndFolders} ${TestFilesAndFolders} ${ProtoFiles})
TARGET_INCLUDE_DIRECTORIES (${ProjectName} PUBLIC             ${CommonPathsLists} ${ProjectPathsLists} ${TestPathsLists} ${PackageIncludePaths} ${ProtoGeneratedPath})
TARGET_LINK_LIBRARIES      (${ProjectName}                    ${PackageNames} gtest::gtest protobuf-c)
TARGET_COMPILE_FEATURES    (${ProjectName} PUBLIC cxx_std_20)

SOURCE_GROUP               (TREE                              ${TestIncludePath} PREFIX "" FILES ${TestFilesAndFolders})
LINK_DIRECTORIES           (${TestsName}                      ${BuildPath}/lib ${PackageLibsPaths})
ADD_EXECUTABLE             (${TestsName}                      ${CommonFilesAndFolders} ${ProjectFilesAndFolders} ${TestFilesAndFolders} ${ProtoFiles})
TARGET_INCLUDE_DIRECTORIES (${TestsName}   PUBLIC             ${CommonPathsLists} ${ProjectPathsLists} ${TestPathsLists} ${ProtoGeneratedPath})
TARGET_LINK_LIBRARIES      (${TestsName}                      ${LibraryName} ${PackageNames} gtest::gtest protobuf-c)
TARGET_COMPILE_FEATURES    (${TestsName}   PUBLIC cxx_std_20)

SOURCE_GROUP               (TREE                              ${ProjectIncludePath} PREFIX "" FILES ${ProjectFilesAndFolders})
ADD_LIBRARY                (${LibraryName} STATIC             ${CommonFilesAndFolders} ${ProjectFilesAndFolders} ${TestFilesAndFolders} ${ProtoFiles})
TARGET_INCLUDE_DIRECTORIES (${LibraryName} PUBLIC             ${CommonPathsLists} ${ProjectPathsLists} ${TestPathsLists} ${PackageIncludePaths} ${ProtoGeneratedPath})
TARGET_LINK_LIBRARIES      (${LibraryName}                    ${PackageNames} gtest::gtest protobuf-c)
TARGET_COMPILE_FEATURES    (${LibraryName} PUBLIC cxx_std_20)
