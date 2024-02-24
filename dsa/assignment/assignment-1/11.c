
    /* deque is we can do insert at beg and del at end */
    /* deque is we can do insert at end and del at beg */
#include<stdio.h>
#include<stdlib.h>
struct node{
    int data;
    struct node *prev;
    struct node *next;
};
struct node *start;
void insert_start(int num){
    struct node *temp;
    temp = (struct node *)malloc(sizeof(struct node));
    temp->data = num;
    temp->prev = NULL;

    if(start == NULL){
        start = temp;
        temp->next = NULL;
    }
    else{
        temp->next = start;
        temp->prev = NULL;
        start->prev = temp;
        start = temp;
    }
}
void insert_end(int num){
    struct node *temp;
    temp = (struct node *)malloc(sizeof(struct node));
    temp->data = num;
    temp->next = NULL;
    if(start == NULL){
        temp->prev = NULL;
        start = temp;
    }
    else{
        struct node *curr;
        curr = start ;
        while(curr->next != NULL){
            curr = curr->next;
        }
        temp->prev = curr;
        curr->next = temp;
        //curr->next->prev = curr;

    }
}
int del_start(){
        int x;
    if(start != NULL){
        struct node *temp;
        temp = start;
        if(start->next == NULL){
            x = temp->data;
            free(temp);
        }
        else{
            x = temp->data;
            start = temp->next;
            free(temp);
        }
    }
    else{
        printf("List is empty\n");
    }
    return x;
}
int del_end(){
        int x;
    if(start != NULL){
        struct node *curr;
        curr = start;
        while(curr->next != NULL){
            curr = curr->next;
        }
        x = curr->data;
        if(curr->prev == NULL){
            start = NULL;
        free(curr);
        }
        else{
            curr->prev->next = NULL;
            curr->prev = NULL;
        free(curr);
        }
    }
    else{
        printf("List is empty");
    }
    return x;
}
void display(){
    struct node *curr;
    curr = start;
    while(curr!= NULL){
        printf("%d ",curr->data);
        curr = curr->next;
    }
    // while(curr != NULL){
    //     printf("%d ",curr->data);
    //     curr = curr->prev;
    // }
}
void enque(int num){
    insert_start(num);
}
int deque(){
    int x;
    x = del_end();
    return x;
}

int main(){
    /* testing purpose */ 
    // insert_start(20);
    // insert_end(40);
    // insert_end(45);
    // insert_end(46);
    // display();
    // int x = del_start();
    // printf("\n%d\n",x);
   // display();
    enque(40);
    enque(30);
    enque(20);
    enque(10);
    display();
    int y = del_end();
//     //del_end();
     printf("\n\ndeleted at end %d\n\n",y);
    //printf("\n%d\n",y);
    display();

}