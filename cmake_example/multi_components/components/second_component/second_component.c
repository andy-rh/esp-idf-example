#include <stdio.h>
#include "third_component.h"

void printf_second_component()
{
    printf("second_component!\n");
    printf_third_component();//在第二个组件中调用第三个组件
}