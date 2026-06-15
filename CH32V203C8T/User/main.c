#include "bsp/bsp.h"
#include "app/app.h"

int main(void) {
    Bsp_Init();
    App_Init();
    App_Loop();
}
