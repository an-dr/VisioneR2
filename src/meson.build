project('VisioneR2', 'c', 'cpp',
        version : '0.0.0',
        license : 'MIT',
        default_options : ['warning_level=3'])
  
include = ['.']
src = [ 
    'main.cpp',
    ]


microlog_dep = dependency('microlog', version : '>=4.0.0')

exe = executable(meson.project_name(), 
                     src, 
                     dependencies : [
                        microlog_dep
                        ],
                     include_directories : include,
                     install : true,
                     )
