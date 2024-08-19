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

sub_proj = cmake.subproject('EmotionCore')
sub_proj = cmake.subproject('object_finder')
microlog_dep = dependency('microlog', version : '>=4.0.0')


exe = executable(meson.project_name(), 
                     src, 
                     dependencies : [
                        microlog_dep
                        ],
                     include_directories : include,
                     install : true,
                     )
