cmake_minimum_required(VERSION 3.22)

project(Plugin1167 VERSION 0.2.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_OSX_DEPLOYMENT_TARGET "11.0" CACHE STRING "")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
option(COPY_PLUGIN_AFTER_BUILD "Copy AU and VST3 bundles after building the plug-in target" OFF)

if(MSVC)
    set(PLUGIN1167_COMPILER_OPTIONS /W4)
else()
    set(PLUGIN1167_COMPILER_OPTIONS -O3 -Wall -Wextra)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# The DSP: the circuit engine (Source/dsp/circuit, from analog-recreation) and the 1176LN model.
# No JUCE dependency, so it is built and tested offline.
file(GLOB_RECURSE PLUGIN1167_DSP_SOURCES CONFIGURE_DEPENDS Source/dsp/*.cpp)
add_library(plugin1167_dsp STATIC ${PLUGIN1167_DSP_SOURCES})
target_include_directories(plugin1167_dsp PUBLIC Source)
target_compile_options(plugin1167_dsp PRIVATE ${PLUGIN1167_COMPILER_OPTIONS})
# The fast engine's hot loop: fused multiply-adds, no errno from the maths functions.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    set_source_files_properties(Source/dsp/Fast1176.cpp PROPERTIES COMPILE_OPTIONS "-ffp-contract=on;-fno-math-errno")
endif()

add_executable(plugin1167_test tests/urei1176_test.cpp)
target_link_libraries(plugin1167_test PRIVATE plugin1167_dsp)
target_compile_options(plugin1167_test PRIVATE ${PLUGIN1167_COMPILER_OPTIONS})

add_executable(plugin1167_stereo_test tests/urei1176_stereo_test.cpp)
target_link_libraries(plugin1167_stereo_test PRIVATE plugin1167_dsp)
target_compile_options(plugin1167_stereo_test PRIVATE ${PLUGIN1167_COMPILER_OPTIONS})

add_executable(plugin1167_eval tests/urei1176_eval.cpp)
target_link_libraries(plugin1167_eval PRIVATE plugin1167_dsp)
target_compile_options(plugin1167_eval PRIVATE ${PLUGIN1167_COMPILER_OPTIONS})

enable_testing()
add_test(NAME plugin1167_test COMMAND plugin1167_test)
add_test(NAME plugin1167_test_fast COMMAND plugin1167_test --fast)
add_test(NAME plugin1167_stereo_test COMMAND plugin1167_stereo_test)

# ---------------------------------------------------------------------------------------------
# PLUG-IN: the JUCE wrapper and editor around urei1176_dsp (two urei1176::Channel instances for
# stereo), its fonts, and the snapshot tool.
# ---------------------------------------------------------------------------------------------
# Prefer a local JUCE checkout for day-to-day development. A fresh clone remains buildable:
# when third_party/JUCE is absent CMake fetches the exact release used by this project.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/JUCE/CMakeLists.txt")
    add_subdirectory(third_party/JUCE)
else()
    include(FetchContent)
    FetchContent_Declare(JUCE
        GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
        GIT_TAG 8.0.9
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(JUCE)
endif()

juce_add_plugin(Plugin1167
    PRODUCT_NAME "1167"
    COMPANY_NAME "codycodie"
    BUNDLE_ID "com.c0dypeng.legacy1167"
    PLUGIN_MANUFACTURER_CODE Cdpg
    PLUGIN_CODE U167
    FORMATS AU VST3
    IS_SYNTH FALSE
    NEEDS_MIDI_INPUT FALSE
    NEEDS_MIDI_OUTPUT FALSE
    VST3_CATEGORIES Fx Dynamics
    AU_MAIN_TYPE kAudioUnitType_Effect
    COPY_PLUGIN_AFTER_BUILD ${COPY_PLUGIN_AFTER_BUILD})

# Inter and JetBrains Mono, both SIL Open Font License (licences are next to the fonts).
juce_add_binary_data(Plugin1167Assets SOURCES
    Resources/fonts/Inter-SemiBold.ttf
    Resources/fonts/JetBrainsMono-Medium.ttf)

target_sources(Plugin1167 PRIVATE
    Source/PluginProcessor.cpp
    Source/PluginEditor.cpp)

target_compile_definitions(Plugin1167 PUBLIC
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
    JUCE_VST3_CAN_REPLACE_VST2=0)

target_link_libraries(Plugin1167
    PRIVATE
        plugin1167_dsp
        Plugin1167Assets
        juce::juce_audio_utils
    PUBLIC
        juce::juce_recommended_config_flags
        juce::juce_recommended_lto_flags
        juce::juce_recommended_warning_flags)

# Development aid: checks the parameter laws and renders the editor to docs/*.png (see
# tests/snapshot.cpp). Run it from the repository root.
juce_add_console_app(plugin1167_snapshot)
target_sources(plugin1167_snapshot PRIVATE
    tests/snapshot.cpp
    Source/PluginProcessor.cpp
    Source/PluginEditor.cpp)
target_compile_definitions(plugin1167_snapshot PRIVATE
    JucePlugin_Name="1167"
    JUCE_MODAL_LOOPS_PERMITTED=1
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0)
target_link_libraries(plugin1167_snapshot
    PRIVATE
        plugin1167_dsp
        Plugin1167Assets
        juce::juce_audio_utils
        juce::juce_recommended_config_flags
        juce::juce_recommended_warning_flags)
