#include<stdio.h>
#include"dec.h"
// struct stack
// {
//     char *c;
//     int size;
//     int tos;
// };
// struct stack sl;
// struct stack s;
int isfull(){
    if(s.tos == s.size-1){
        return 1;
    }
    return 0;
}
int isempty(){
    if(s.tos == -1){
        return 1;
    }
    return 0;
}
void push(char ch){
    if(!isfull()){
        s.tos++;
        s.c[s.tos] = ch;
       // printf("\nPush stack: %d",s.c[s.tos]);
    }
    else{
        printf("dont push\n");
    }
}
int pop(){
    int x;
    if(!isempty()){
        x = s.c[s.tos];
        s.tos--;
    }
    else{
        printf("don't pop\n");
    }
    return x;
}
int peep(){
    int x;
    if(!isempty()){
        x = s.c[s.tos];
    }
    return x;
}