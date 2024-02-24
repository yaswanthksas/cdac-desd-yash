#include <stdio.h>
#include <stdlib.h>
struct node
{
    int data;
    struct node *next;
};
struct node *start;
void insert_start(int num)
{
    struct node *temp;
    temp = (struct node *)malloc(sizeof(struct node));
    temp->data = num;
    if (start == NULL)
    {
        start = temp;
        temp->next = NULL;
    }
    else
    {
        temp->next = start;
        start = temp;
    }
}
void insert_end(int num)
{
    struct node *curr, *temp;
    temp = (struct node *)malloc(sizeof(struct node));
    temp->data = num;
    temp->next = NULL;
    curr = start;
    while (curr->next != NULL)
    {
        curr = curr->next;
    }
    curr->next = temp;
}
void delete_spec_insert_front(int num)
{   int found = 0;
    if (start != NULL)
    {
        struct node *temp, *curr;
        curr = start;
        if (start->data == num)
        {   found = 1;
            start = start->next;
            free(curr);
        }
        else
        {
            while (curr->next != NULL && curr->next->data != num)
            {
                curr = curr->next;
            }
            if (curr->next != NULL)
            {   found = 1;
                temp = curr->next;
                curr->next = temp->next;
                // temp = temp->next;
                temp->next = NULL;
                free(temp);
            }
            else{
                printf("Element not found\n");
            }
        }
    }
        else
        {
            printf("List is empty\n");
        }
    if(found == 1){
        insert_start(num);
    }
    
}
void display()
{
    struct node *curr;
    curr = start;
    while (curr != NULL)
    {
        printf("%d ", curr->data);
        curr = curr->next;
    }
    printf("\n");
}
int main()
{
    insert_start(20);
    insert_start(40);
    insert_start(60);
    insert_end(80);
    display();
    int find;
    printf("Entering it to find it first\n");
    scanf("%d", &find);
    delete_spec_insert_front(find);
    display();
    return 0;
}