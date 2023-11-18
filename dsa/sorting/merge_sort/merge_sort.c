#include <stdio.h>
void merge(int arr[],int low,int mid,int high){
    int b[high+1];
    int i,j,k;
    i = low;
    j = mid+1;
    k = low;
    while(i<=mid && j<=high){
        if(arr[i] <= arr[j]){
            b[k] = arr[i];
            i++;
            k++;
        }
        else{
            b[k] = arr[j];
            j++;
            k++;
        }
    }
    if(i>mid){
        while(j<=high){
            b[k] = arr[j];
            j++;
            k++;
        }
    }
    //if(j>high){
    else{
        while(i<=mid){
            b[k] = arr[i];
            i++;
            k++;
        }
    }
     for(int i=low;i<k;i++){
        //printf("%d ",arr[i]);
        arr[i] = b[i];
    }
}
void mergesort(int arr[], int low, int high)
{
    int mid;
    if (low < high)
    {
        mid = (low + high) / 2;
        mergesort(arr, low, mid);
        mergesort(arr, mid + 1, high);
        merge(arr, low, mid, high);
        
    }
   
}

int main()
{
    // int n;
    // printf("Enter size of n");
    int arr[10] = {8, 9, 5, 45, 85, 12, 7, 35, 51, 20};
    int low = 0;
    int high = 9;
    mergesort(arr, low, high);

    for(int i=0;i<10;i++){
        printf("%d ",arr[i]);
    }
     

    return 0;
}