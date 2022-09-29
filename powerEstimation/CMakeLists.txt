

cmake_minimum_required(VERSION 2.6)

project(tester)


add_executable(tester ModelImporter.h ModelImporter.cpp main.cpp)


add_executable(testInstructionModel InstructionModel.h InstructionModel.cpp CSVReader.h TestInstructionModel.cpp)