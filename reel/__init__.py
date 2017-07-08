#!/usr/bin/env python3

class CodeBuilder:
    pass

class CmakeBuilder(CodeBuilder):
    pass

class AutotoolsBuilder(CodeBuilder):
    pass

class MakeBuilder(CodeBuilder):
    pass

class AutoBuilder(CodeBuilder):
    pass

class URLDownloader:
    pass

class GitDownloader:
    pass

class AutoDownloader:
    pass

class Platform:
    def __init__(self):
        pass

    def add_library():
        # TODO add to our list to build
        pass

class Reel:
    def __init__(self):
        self.platforms = []

    def add_platform(self):
        return Platform()

    def add_tool(self):
        # TODO add to our list to build for native

    def build(self):

        # http://preshing.com/20141119/how-to-build-a-gcc-cross-compiler/

        # TODO go build our compiler for our native platform

        # TODO go build all our libraries needed

        # TODO go self host our compiler

        # TODO go build our tools

        # TODO for each of our platforms build our cross compiler

        # TODO for each of our platforms use the cross compilers to build their libraries
