import os

from conan             import ConanFile
from conanPackages     import conanPackages
from conan.tools.files import copy, load
from conan.tools.cmake import CMake, cmake_layout

class Conan(ConanFile):
    name            = "template"
    version         = "x.x"
    user            = "ssitkowx"
    channel         = "stable"
    license         = "freeware"
    repoUrl         = "https://github.com/ssitkowx"
    url             = repoUrl + '/' + name + '.git'
    description     = "General class for gpio"
    settings        = "os", "compiler", "build_type", "arch"
    options         = { "shared": [True, False] }
    default_options = { "shared": False         }
    generators      = "CMakeDeps", "CMakeToolchain"
    author          = "sylsit"
    exports         = "*"
    requires        = ["gtest/cci.20210126"]
    downloadPath    = "/home/sylwester/.conan2/download"
    repoPath        = downloadPath + '/repos'
    packagePath     = downloadPath + '/packages'
    packages        = []

    def layout (self):
        projectPath = os.getcwd ().replace ('/Conan','')
        cmake_layout (self, src_folder = projectPath, build_folder = projectPath + '/Build')

    def source (self):
        cmake_file = load (self, "CMakeLists.txt")
        conanPackages.install (self, self.downloadPath, self.repoPath, self.repoUrl, self.packages)

    def build (self):
        if self.settings.os == 'Linux' and self.settings.compiler == 'gcc':
            cmake = CMake (self)
            cmake.configure ()
            cmake.build     ()
        else:
            raise Exception ('Unsupported platform or compiler')

    def package (self):
        packagePath = self.packagePath + '/' + self.name

        copy (self, '*.h'  , src = os.path.join (self.source_folder, "Project"), dst = os.path.join (packagePath, "include")        , keep_path = False)
        copy (self, '*.hpp', src = os.path.join (self.source_folder, "Project"), dst = os.path.join (packagePath, "include")        , keep_path = False)
        copy (self, '*.hxx', src = os.path.join (self.source_folder, "Project"), dst = os.path.join (packagePath, "include")        , keep_path = False)
        copy (self, '*.a'  , src = self.build_folder                           , dst = os.path.join (packagePath, "lib")            , keep_path = False)

        copy (self, '*.h'  , src = os.path.join (self.source_folder, "Project"), dst = os.path.join (self.package_folder, "include"), keep_path = False)
        copy (self, '*.hpp', src = os.path.join (self.source_folder, "Project"), dst = os.path.join (self.package_folder, "include"), keep_path = False)
        copy (self, '*.hxx', src = os.path.join (self.source_folder, "Project"), dst = os.path.join (self.package_folder, "include"), keep_path = False)
        copy (self, '*.a'  , src = self.build_folder                           , dst = os.path.join (self.package_folder, "lib")    , keep_path = False)

    def export_sources (self):
        receipePath = os.path.join (self.recipe_folder, "..")

        copy (self, "*.txt"        , receipePath, self.export_sources_folder)
        copy (self, "Tests/*.hpp"  , receipePath, self.export_sources_folder)
        copy (self, "Tests/*.hxx"  , receipePath, self.export_sources_folder)
        copy (self, "Tests/*.cxx"  , receipePath, self.export_sources_folder)
        copy (self, "Project/*.h"  , receipePath, self.export_sources_folder)
        copy (self, "Project/*.hpp", receipePath, self.export_sources_folder)
        copy (self, "Project/*.cpp", receipePath, self.export_sources_folder)
