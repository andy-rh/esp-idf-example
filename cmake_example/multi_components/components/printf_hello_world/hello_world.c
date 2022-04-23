#include <stdio.h>
#include "second_component.h"

void printf_hello_world()
{
    printf("Hello world!\n");
    printf_second_component();//第二个组件
}