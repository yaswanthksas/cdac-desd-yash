        // add two sorted lists 
#include<stdio.h>
#include<stdlib.h>
struct node{
    int data;
    struct node *next;
};
struct node *l1,*l2;
struct node * insert(struct node *l1,int num){
    struct node *temp,*curr;
    temp = (struct node *)malloc(sizeof(struct node));
    temp->data = num;
    temp->next = NULL;
    if(l1==NULL){
        l1 = temp;
    }
    else{
        curr = l1;
        while(curr->next != NULL){
            curr=curr->next;
        }
        curr->next = temp;

    }
    return l1;
}
void display(struct node *l1){
    struct node *curr;
    curr = l1;
    while(curr != NULL){
        printf("%d ",curr->data);
        curr=curr->next;
    }
    // return l1;
}
struct node * mergell(struct node *l1,struct node *l2){

    if(l1 == NULL){
        return l2;
    }
    if(l2 == NULL){
        return l1;
    }
    struct node *l3,*curr;
    l3 = (struct node *)malloc(sizeof(struct node));
    curr = l3;
    while(l1 != NULL && l2 != NULL){
        if(l1->data < l2->data){
            curr->next = l1;
            l1 = l1->next;
        }
        else{
            curr->next = l2;
            l2 = l2->next;
        }
        curr=curr->next;
    }
    if(l1 != NULL){
        curr->next = l1;
    }
    if(l2 != NULL){
        curr->next = l2;
    }

    return l3->next;
}
int main(){
    //here we can take our own input like before 

    l1= insert(l1,20);
    //printf("%d",l1->data);
    l1=insert(l1,40);
    // printf("%d",l1->data);
    l1=insert(l1,60);
    // printf("%d",l1->data);
    display(l1);

    printf("\n");

    l2 = insert(l2,10);
    l2 = insert(l2,30);
    l2 = insert(l2,50);

    display(l2);
    printf("\n");
    struct node *l3;
    l3 = mergell(l1,l2);
    display(l3);


    return 0;
}