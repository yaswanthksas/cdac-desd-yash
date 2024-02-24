#include <stdio.h>
#include "dec.h"
#include <stdlib.h>
#include <string.h>
// struct stak
// {
//     int *ar;
//     int size;
//     int tos;
// };
// struct stak s;

char str[] = "142*4/+56*3/-";
int main()
{
    int l = strlen(str);
    char ch;
    int a, b;
    s.c = (char *)malloc(sizeof(int) * l);
    s.size = l;
    s.tos = -1;
    for (int i = 0; i < l; i++)
    {
        ch = str[i];
        switch (ch)
        {
        case '+':
            a = pop();
            b = pop();
            push(b + a);
            break;
        case '-':
            a = pop();
            b = pop();
            push(b - a);
            break;
        case '*':
            a = pop();
            b = pop();
            push(b * a);
            break;
        case '/':
            a = pop();
            b = pop();
            push(b / a);
            break;
        default:
            push(atoi(&ch));
            break;
        }
    }
    int result;
    result = pop();
    printf("%d", result);
    return 0;
}