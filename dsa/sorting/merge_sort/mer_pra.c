#include<stdio.h>
void merge(int arr[],int low,int mid,int high){
    int i=low;
    int j=mid+1;
    int b[high+1];
    int k=low;
    while(i<=mid && j<=high){
        if(arr[i]<=arr[j]){
            b[k] = arr[i];
            i++;
            k++;
        }
        else if(arr[j] < arr[i]){
            b[k] = arr[j];
            j++;
            k++;
        }
    }
    while(i <= mid){
        b[k] = arr[i];
        i++;
        k++;
    }
    while(j<=high){
        b[k] = arr[j];
        j++;
        k++;
    }
    for(int i=low;i<=high;i++){
        arr[i] = b[i];
    }
}
void mergesort(int arr[],int low,int high){
    if(low<high){
        int mid;
        mid = (low+high)/2;
        mergesort(arr,low,mid);
        mergesort(arr,mid+1,high);
        merge(arr,low,mid,high);
    }
}

int main(){
    int arr[9] = {15,5,24,8,1,3,16,10,20};
    int low,high;
    low = 0;
    high = 8;
    mergesort(arr,low,high);
    for(int i=0;i<high+1;i++){
        printf("%d ",arr[i]);
    }
    return 0;
}