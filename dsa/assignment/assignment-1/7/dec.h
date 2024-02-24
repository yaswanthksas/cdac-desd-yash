void push(char num);
int pop();
int peep();
int isempty();
int isfull();
struct stack
{
    char *c;
    int size;
    int tos;
};
struct stack s;