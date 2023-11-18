#include<stdio.h>

int main(){
    int arr[] = {3,4,5,2,1};
    int n = 5;
    for(int i=0;i<n-1;i++){   //4 passes
        int min = i;    
        for(int j=i+1;j<n;j++){
            if(arr[j]<arr[min]){
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
    for(int i=0;i<n;i++){
        printf("%d",arr[i]);
    }

    return 0;
}