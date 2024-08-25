project('VisioneR2', 'c', 'cpp',
        version : '0.0.0',
        license : 'MIT',
        default_options : ['warning_level=3'])
  
include = ['.']
src = [ 
    'main.cpp',
    ]

add_global_arguments('-DULOG_HAVE_TIME', language : 'c')
add_global_arguments('-DULOG_HIDE_FILE_STRING', language : 'c')
add_project_arguments('-fmacro-prefix-map=../=',language: 'c')

cmake = import('cmake')

# sub_proj = cmake.subproject('EmotionCore')
# sub_proj = cmake.subproject('object_finder')
remotion_ctrl_proj = cmake.subproject('remotion_ctrl')
libv4l2cpp_dep = remotion_ctrl_proj.get_variable('libv4l2cpp_dep')
CppLinuxSerial_dep = remotion_ctrl_proj.get_variable('CppLinuxSerial_dep')


exe = executable(meson.project_name(), 
                     src, 
                     dependencies : [
                        dependency('microlog', version : '>=4.0.0'),
                        remotion_ctrl_proj.dependency('remotion_ctrl'),
                        dependency('opencv4'),
                        libv4l2cpp_dep,
                        CppLinuxSerial_dep
                        ],
                     include_directories : include,
                     install : true,
                     )
