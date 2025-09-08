CMAKE_MINIMUM_REQUIRED (VERSION 3.16)
PROJECT                (template)

SET (ProjectName       Project)
SET (TestsName         Tests)
SET (LibraryName       ${CMAKE_PROJECT_NAME})
SET (CurrentPath       ${CMAKE_CURRENT_SOURCE_DIR})
SET (PackageNames )
SET (PackageIncludePath )

MACRO (LOG_ERROR msg)
    EXECUTE_PROCESS (COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --red --bold ${msg})
ENDMACRO ()

SET  (ProjectIncludePath                  ${CurrentPath}/${ProjectName})
FILE (GLOB_RECURSE ProjectFilesAndFolders ${ProjectIncludePath}/*)

SET  (TestIncludePath                     ${CurrentPath}/${TestsName})
FILE (GLOB_RECURSE TestFilesAndFolders    ${TestIncludePath}/*)

MACRO (PARSE_SUBDIRECTORIES_PATHS search_directory paths)
    SET (directoryPaths "")
    FOREACH (subdirectoryPath ${search_directory})
        GET_FILENAME_COMPONENT (directoryPath ${subdirectoryPath} PATH)
        SET (directoryPaths ${directoryPaths} ${directoryPath})
    ENDFOREACH ()

    LIST (REMOVE_DUPLICATES directoryPaths)
    SET  (${paths} ${directoryPaths})
ENDMACRO ()

PARSE_SUBDIRECTORIES_PATHS ("${ProjectFilesAndFolders}" ProjectPathsLists)
PARSE_SUBDIRECTORIES_PATHS ("${TestFilesAndFolders}"    TestPathsLists)

MACRO (GET_PACKAGE_PATHS path folderName result)
    FOREACH (packageName ${PackageNames})
        STRING (CONCAT newPath "${path}" "/${packageName}/${folderName}")
        LIST                     (APPEND ${result} ${newPath})
    ENDFOREACH (packageName)
ENDMACRO()

GET_PACKAGE_PATHS ("${PackageIncludePath}" "lib"     PackageLibsPaths)
GET_PACKAGE_PATHS ("${PackageIncludePath}" "include" PackageIncludePaths)

ENABLE_TESTING             ()
FIND_PACKAGE               (GTest REQUIRED)

SOURCE_GROUP               (TREE                              ${ProjectIncludePath} PREFIX "" FILES ${ProjectFilesAndFolders})
LINK_DIRECTORIES           (${ProjectName}                    ${PackageLibsPaths})
ADD_EXECUTABLE             (${ProjectName}                    ${ProjectFilesAndFolders} ${TestFilesAndFolders})
TARGET_INCLUDE_DIRECTORIES (${ProjectName} PUBLIC             ${ProjectPathsLists} ${TestPathsLists} ${PackageIncludePaths})
TARGET_LINK_LIBRARIES      (${ProjectName}                    ${PackageNames} gtest::gtest)
TARGET_COMPILE_FEATURES    (${ProjectName} PUBLIC cxx_std_17)

SOURCE_GROUP               (TREE                              ${TestIncludePath} PREFIX "" FILES ${TestFilesAndFolders})
LINK_DIRECTORIES           (${TestsName}                      ${BuildPath}/lib ${PackageLibsPaths})
ADD_EXECUTABLE             (${TestsName}                      ${ProjectFilesAndFolders} ${TestFilesAndFolders})
TARGET_INCLUDE_DIRECTORIES (${TestsName}   PUBLIC             ${ProjectPathsLists} ${TestPathsLists})
TARGET_LINK_LIBRARIES      (${TestsName}                      ${LibraryName} ${PackageNames} gtest::gtest)
TARGET_COMPILE_FEATURES    (${TestsName}   PUBLIC cxx_std_17)

SOURCE_GROUP               (TREE                              ${ProjectIncludePath} PREFIX "" FILES ${ProjectFilesAndFolders})
ADD_LIBRARY                (${LibraryName} STATIC             ${ProjectFilesAndFolders} ${TestFilesAndFolders})
TARGET_INCLUDE_DIRECTORIES (${LibraryName} PUBLIC             ${ProjectPathsLists} ${TestPathsLists} ${PackageIncludePaths})
TARGET_LINK_LIBRARIES      (${LibraryName}                    ${PackageNames} gtest::gtest)
TARGET_COMPILE_FEATURES    (${LibraryName} PUBLIC cxx_std_17)
