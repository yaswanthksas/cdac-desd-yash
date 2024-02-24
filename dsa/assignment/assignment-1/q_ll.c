#include<stdio.h>
#include<stdlib.h>
struct node{
    int data;
    struct node *next;
};
struct node *start;
struct node * insert_end_enque(struct node *start ,int num){
    struct node *temp;
    temp = (struct node *)malloc(sizeof(struct node));
    temp->data = num;
    temp->next = NULL;
    if(start == NULL){
        start = temp;
    }
    else{
        struct node *curr;
        curr = start;
        while(curr->next != NULL){
            curr = curr->next;
        }
        curr->next = temp;
    }
    return start;
}
struct node * delete_first_deque(struct node *start){
    int x;
    if(start != NULL){
        struct node *temp;
        temp = start;
        x = temp->data;
        start = start->next;
        printf("deleted data : %d\n",x);
        free(temp);
        temp = NULL;

    }
    else{
        printf("List is empty\n");
    }
    return start;

}
void display(){
    struct node *curr;
    curr = start;
    while(curr != NULL){
        printf("%d ",curr->data);
        curr = curr->next;
    }
    printf("\n");
}
int main(){
    start = insert_end_enque(start,20);
    start = insert_end_enque(start,10);
    display();
    //int x;
    start = delete_first_deque(start);

    display();

    return 0;
}