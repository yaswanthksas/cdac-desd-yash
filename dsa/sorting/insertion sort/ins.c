#include<stdio.h>
int main(){
    int arr[] = {3,5,4,1,2};
    int n = 5;
    for(int i=1;i<n;i++){
        int temp,j;
        temp = arr[i];
        j = i-1;
        while(j>=0 && arr[j] > temp){
            arr[j+1] = arr[j];
            j--;
        }
        arr[j+1] = temp;
    }

    printf("Display\n");
    for(int i=0;i<n;i++){
        printf("%d ",arr[i]);
    }

    return 0;
}