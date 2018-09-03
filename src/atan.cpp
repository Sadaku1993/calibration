#include <stdio.h>
#include <math.h>

#define PI 3.141592

int main(void)
{
    float x;
    float y;

    printf("x:");
    scanf("%f", &x);

    printf("y:");
    scanf("%f", &y);

    double rad;
    rad = atan2(y, x);

    double dig = rad / PI * 180;

    printf("x:%5.2f y:%5.2f rad:%5.2f dig:%5.2f\n",x, y, rad, dig);

    return 0;
}
