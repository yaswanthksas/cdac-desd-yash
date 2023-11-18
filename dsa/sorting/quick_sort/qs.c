#include<stdio.h>
int partition(int arr[],int low,int high){
    int pivot,i,j,temp;
    pivot = arr[high];
    i = low-1;
    for(j=low;j<=high;j++){
        if(arr[j] <= pivot){
            i++;
            temp = arr[i];
            arr[i] = arr[j];
            arr[j] = temp;
        }
    }
    // int x;
    // x = arr[i+1];
    // arr[i+1] = pivot;
    // pivot = x;
    return (i);
}
void quicksort(int arr[],int low,int high){
    if(low<high){
        int p;
        p = partition(arr,low,high);
        quicksort(arr,low,p-1);
        quicksort(arr,p+1,high);
    }
}
int main(){
    int arr[] = {15,5,23,7,18,9,76,32,4,10};
    int n;
    n = 10;
    int low,high;
    low = 0;
    high = n-1;
    quicksort(arr,low,high);

    for(int i=0;i<n;i++){
        printf("%d ",arr[i]);
    }


    return 0;
}