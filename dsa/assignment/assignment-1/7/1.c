#include<stdio.h>
#include"dec.h"
#include<string.h>
#include<stdlib.h>
char str[] = "{a+b-[d*e/g]}";
int main(){
    int l= strlen(str);
    s.c = (char *)malloc(sizeof(char)*l);
    s.size = l;
    s.tos = -1;
    int i,flag = 0;
    char ch,a;
    for(i=0;i<l;i++){
        ch = str[i];
        switch (ch)
        {
        case '{':
        case '[':
        case '(':
            push(ch);
            break;
        case ']':
        case ')':
        case '}':
            if((ch == ']' && peep() == '[') || (ch == ')' && peep() == '(') || (ch == '}' && peep() == '{')){
                a = pop();
            }
            else{
                flag = 1;
                break;
            }
        default:
            ;
        }
        if(flag == 1){
            break;
        }
    }
    if(i == l && isempty()){
        printf("Balanced");
    }
    else{
        printf("not balanced");
    }
    return 0;
}