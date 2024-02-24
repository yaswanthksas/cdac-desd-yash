#include<stdio.h>
#include<stdlib.h>
struct cque{
    int *c;
    int size;
    int rear;
    int front;
};
struct cque cq;
int underflow(){
    if(cq.rear == -1 && cq.front == -1){
        return 1;
    }
    return 0;
}
int overflow(){
    if((cq.rear+1)%cq.size == cq.front){
        return 1;
    }
    return 0;
}
void enque(int num){
    if(!overflow()){
        if(underflow()){
            cq.front++;
        }
        cq.rear = (cq.rear+1)%cq.size;
        cq.c[cq.rear] = num;
    }
    else{
        printf("Queue is empty");
    }
}
int deque(){
    int x = -999;
    if(!underflow()){
        x = cq.c[cq.front];
        if(cq.front == cq.rear){
            cq.front = -1;
            cq.rear = -1;
        }
        //else{
            cq.front = (cq.front+1)%cq.size;
        
    }
    return x;

}
void display(){
    int i;
    if(!underflow()){
        for(i= cq.front;i != cq.rear; i=(i+1)%cq.size){
            printf("%d ",cq.c[i]);
        }
        printf("%d ",cq.c[i]);
    }
    else{
        printf("list is empty\n");
    }
}
int main(){
    printf("size:\n");
    scanf("%d",&cq.size);
    cq.c = (int *)malloc(sizeof(int)*cq.size);
    cq.rear = cq.front = -1;
    int ch,num,x;
    while(1){
        printf("1. Enque\n ");
        printf("2. deque\n ");
        printf("3. display\n ");
        printf("4. Exit\n ");
        printf("Enter choice\n");
        scanf("%d",&ch);
        switch(ch){
            case 1:
                printf("Enter number to insert\n");
                scanf("%d",&num);
                enque(num);
                break;
            case 2:
                x = deque();
                printf("deleting element %d\n",x);
                break;
            case 3:
                 display();
                //printf("deleting element%d\n",x);
                break;
            case 4:
                exit(1);
            default:
                printf("Enter other elemnt");
        }
    }
    // enque(20);
    // enque(50);
    // display();


    return 0;
}