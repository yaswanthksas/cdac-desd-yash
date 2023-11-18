#include<stdio.h>
// n-1 passes
// find minimum, place it first  // comparison
// which is swapping 

void sel_sort(int *arr,int n){
    for(int i=0;i<n-1;i++){
        int min = i;
        for(int j=i+1;j<n;j++){
            if(arr[j] < arr[min]){
                min = j;
            }
        }
        if(min != i){
            int temp;
            temp = arr[i];
            arr[i] = arr[min];
            arr[min] = temp;
        }
    }
}
void display(int *arr,int n){
    for(int i=0;i<n;i++){
        printf("%d",arr[i]);
    }
    printf("\n");
}
int main(){
    int n;
    printf("enter size of array:\n");
    scanf("%d",&n);
    int arr[n];
    printf("Entering the array elements :\n");
    for(int i=0;i<n;i++){
        scanf("%d",&arr[i]);
    }
    printf("Unsorted array :\n");
    // for(int i=0;i<n;i++){
    //     printf("%d",arr[i]);
    // }
    display(arr,n);
    // int min;
    printf("Sorted array is \n");
    sel_sort(arr,n);
    // for(int i=0;i<n-1;i++){   // three passes 
    //     min = i;
    //     for(int j=i+1;j<n;j++){
    //         if(arr[j]<arr[min]){
    //             min = j;
    //         }
    //     }
    //     if( min != i){
    //         int temp;
    //         temp = arr[i];
    //         arr[i] = arr[min];
    //         arr[min] = temp;
    //     }

    // }
    // for(int i=0;i<n;i++){
    //     printf("%d",arr[i]);
    // }
    display(arr,n);


    return 0;
}