#include<stdio.h>
#include<stdlib.h>
struct node{
    int data;
    struct node *next;
};
struct node *front,*rear;
void insert_end_enque(int num){
    struct node *temp;
    temp = (struct node *)malloc(sizeof(struct node));
    temp->data = num;
    temp -> next = NULL;
    if(front == NULL){
        front = temp;
    }
    else{
        rear->next = temp;
    }
    rear = temp;
}
int underflow(){
    if(front == NULL){
        return 1;
    }
    return 0;
}
int delete_front_deque(){
    struct node *temp;
    temp = front;
    if(temp == NULL){
        printf("list is empty\n");
        return;
    }
    int x;
    x = temp->data;
    front = front->next;
    free(temp);
    return x;
}
int peep(){
    if(underflow()){
        printf("list is empty\n");
    }
    return front->data;
}
void display(){
    struct node *curr;
    curr = front;
    if(underflow()){
        printf("list is empty\n");
    }
    else{
        while(curr != NULL){
            printf("%d ",curr->data);
            curr = curr->next;
        }
        printf("\n");
    }
}
int main(){
    while(1){
        int choice,num;
        printf("1. Insert a element in queue \n");
        printf("2. delete a element in queue \n");
        printf("3. peep element \n");
        printf("4. display the queue \n");
        printf("5. exit \n");
        printf("Enter your choice\n");
        scanf("%d",&choice);
        switch(choice){
            case 1: 
                printf("Element you want to insert \n");
                scanf("%d",&num);
                insert_end_enque(num);
                break;
            
            case 2:
                printf("Deleted element is %d",delete_front_deque());
                break;
            
            case 3:
                printf("Item at front of queue %d:",peep());
                break;
            
            case 4:
                display();
                break;

            case 5:
                exit(1);
        }
    }
    // insert_end_enque(20);
    // insert_end_enque(10);
    // insert_end_enque(30);
    // insert_end_enque(40);
    // display();
    // int x,y;
    // x = delete_front_deque();
    // y = delete_front_deque();
    // display();

    return 0;
}