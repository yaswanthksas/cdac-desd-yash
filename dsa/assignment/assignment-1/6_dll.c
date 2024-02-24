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
    temp = (struct node *)malloc(sizeof(struct node *));
    temp->data = num;
    if(start == NULL){
        start = temp;
        temp->prev = NULL;
        temp->next = NULL;
    }
    else{
        temp->prev = NULL;
        temp->next = start;
        start = temp;
    }
    //return temp;
}

void insert_end(int num){
    struct node *temp;
    temp = (struct node *)malloc(sizeof(struct node *));
    temp->data = num;
    temp->next = NULL;
    if(start == NULL){
        temp->prev = NULL;
        start = temp;
    }
    else{
        struct node *curr;
        curr = start;
        while(curr->next != NULL){
            curr = curr->next;
        }
        curr->next = temp;
        curr->next->prev = curr;    //temp->prev;
    }
}
void insert_after(int ele,int num){
    struct node *curr;
    curr = start;
    if(start != NULL){
        while(curr != NULL && curr->data != ele){
            curr = curr->next;
        }
        if(curr != NULL){
            struct node *temp;
            temp = (struct node *)malloc(sizeof(struct node));
            temp->data = num;
            temp->next = curr->next;
            temp->prev = curr;
            if(curr->next != NULL){
                curr->next->prev = temp;
            }
            curr->next = temp;
        }
        else{
            printf("Element Not found\n");
        }
    }
    else{
        printf("List is empty\n");
    }
}
void insert_before(int ele,int num){
    struct node *curr;
    curr = start;
    if(start != NULL){
        while(curr != NULL && curr->data != ele){
            curr = curr->next;
        }
        if(curr != NULL){
            struct node *temp;
            temp = (struct node *)malloc(sizeof(struct node));
            temp->data = num;
            temp->next = curr;
            temp->prev = curr->prev;
            if(curr->prev != NULL){
                curr->prev->next = temp;
            }
            else{
                start = temp;
            }
            curr->prev = temp;
        }
        else{
            printf("Element not found\n");
        }
    }
    else{
        printf("List is empty\n");
    }
}
int delete_first(){
    if(start != NULL){ 
        struct node *temp;
        temp = start;
        int x;
        x = temp->data;
        start = start->next;
        if(start != NULL){
            start->prev = NULL;
        }
        free(temp);
        return x;
    }
    else{
        printf("List is empty\n");
    }
}

int delete_last(){
    if(start != NULL){
        struct node *curr;
        curr = start;
        while(curr->next != NULL){
            curr = curr->next;
        }
        int x;
        x = curr->data;
        if(curr->prev == NULL){
            start = NULL;
        }
        else{
            curr->prev->next = NULL;
            curr->prev = NULL;
        }
    return x;
        free(curr);
    }
    else{
        printf("List is empty\n");
    }
}
void del_spec(int ele){
    if(start != NULL){
        struct node *curr;
        curr = start;
        while(curr != NULL && curr->data != ele){
            curr = curr->next;
        }
        if(curr != NULL){
            int x;
            x = curr->data;
            if(curr->next != NULL){
                curr->next->prev = curr->prev;
            }
            if(curr->prev != NULL){
                curr->prev->next = curr->next;
            }
            else{
                start = curr->next;
            }
            free(curr);
        }
        else{
            printf("Element not found\n");
        }
    }
    else{
        printf("List is empty\n");
    }
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
void traverse_backward(){
    struct node *curr;
    curr = start;
    if(start != NULL){
        while(curr->next != NULL){
        curr = curr->next;
        }
        while(curr != NULL){
            printf("%d ",curr->data);
            curr = curr->prev;
        }
    }
    else{
        printf("List is empty\n");
    }
    

}
int main(){
    int num,n;
        int ele,elem,elex,eley;
    while(1){
        printf("1. Insert at start: \n");
        printf("2. Insert at end: \n");
        printf("3. Insert at after specified: \n");
        printf("4. Insert at before: \n");
        printf("5. delete at start: \n");
        printf("6. delete at end: \n");
        printf("7. delete at specified: \n");
        printf("8. display elements : \n");
        printf("9. traverse backward: \n");
        printf("10. exit: \n\n");
        printf("Enter any number \n");
        scanf("%d",&num);
        switch(num){
            case 1:
                printf("Enter number to insert first\n");
                scanf("%d",&n);
                insert_start(n);
                break;
            
            case 2:
                printf("Enter number to insert end\n");
                scanf("%d",&n);
                insert_end(n);
                break;
            
            case 3:
                printf("Enter number to insert after\n");
                scanf("%d",&n);
                insert_after(elem,n);
                break;
            
            case 4:
               // int ele;
                printf("Enter number to insert before\n");
                scanf("%d",&n);
                insert_after(ele,n);
                break;
            
            case 5:
                //int ele;
                // printf("Enter number to delete first\n");
                // scanf("%d",&n);
                elex = delete_first();
                printf("Element deleted at %d\n",elex);
                break;
            
            case 6:
                //int ele;
                // printf("Enter number to delete last\n");
                // scanf("%d",&n);
                eley = delete_last();
                printf("Element deleted at %d\n",eley);
                break;
            
            case 7:
                
                printf("Enter number to delete specified\n");
                scanf("%d",&n);
                del_spec(n);
                printf("Element deleted at %d\n",ele);
                break;

            case 8:
                display();
                break;
            
            case 9:
                traverse_backward();
                break;
            
            case 10:
                exit(1);
            
            // default:
            //     printf("thankyou\n");
        }
    }
    // insert_start(20);
    // insert_start(40);
    // insert_end(12);
    // insert_after(40,89);
    // insert_before(89,60);
    // display();
    // int x;
    // x = delete_first();
    // printf("\n%d\n",x);
    // display();
    // int y;
    // y = delete_last();
    // printf("\n%d\n",y);
    // del_spec(89);
    // display();
    // traverse_backward();

    return 0;
}