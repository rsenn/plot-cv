xx () 
{ 
    for x in  circle contour line polygon rect;
    do
        y=$(str_camelize "${x#js_draw_}");
        z=draw$y
        cat  <<EOF
            JS_CFUNC_DEF("draw$y", 1, &js_draw_$x),
EOF

    done
}


yy () 
{ 
    for x in contour draw mat point point_iterator rect size
    do
        y=$(str_toupper "${x}");
        cat  <<EOF
#if defined(JS_${y}_MODULE) || defined(quickjs_${x}_EXPORTS)
#define JS_INIT_MODULE js_init_module
#else
#define JS_INIT_MODULE js_init_module_${x}
#endif

EOF

    done
}
