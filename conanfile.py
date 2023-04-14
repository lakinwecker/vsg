from conan.tools.cmake import CMake
from conan import ConanFile


class Recipe(ConanFile):
    v = open("version.txt").readline().strip()
    name = "vsg"
    version = v

    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps", "VirtualRunEnv"

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports = "version.txt"
    exports_sources = "version.txt", "CMakeLists.txt", "CMakePresets.json", "cmake/*", "include/*", "source/*"

    def layout(self):
        self.folders.generators = "conan"

    def requirements(self):
        requirements = [
            "immer/0.8.0",
            "doctest/2.4.10",
        ]
        for r in requirements:
            self.requires(r)

    def build_requirements(self):
        pass

    def package_info(self):
        self.cpp_info.libs = []

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
