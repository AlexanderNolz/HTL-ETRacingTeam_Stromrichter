#include <stdio.h>
#include "Regelungstechnik_lib/Regelungstechnik.h"
#include <windows.h>


int main() {
    PI PI_Glied1;
    float x=0.1f;
    float y;
    config_PID(&PI_Glied1,0.1f,12.0f);

    for (int i = 0; i < 100; ++i) {
        y=100.0f-x;
        add_val_PID(&PI_Glied1,y,0.001f);
        x=PI_Glied1.val/0.2f;
        Sleep(1);

        printf("%f \n" ,x);
    }


    return 0;
}

