#include<stdio.h>
int partition(int arr[],int l,int h){
    int pivot = arr[h];
    int i = l-1,temp,x;
    for(int j=l;j<h;j++){
        if(arr[j]<=pivot){
            i++;
            temp = arr[i];
            arr[i] = arr[j];
            arr[j] = temp;
        }
    }
    x = arr[i+1];
    arr[i+1] = pivot;
    pivot = x;
    return i+1;
}
void quicksort(int arr[],int l,int h){
    if(l<=h){
        int q;
        q = partition(arr,l,h);
        quicksort(arr,l,q-1);
        quicksort(arr,q+1,h);
    }
}

int main(){
   int n;
   printf("Enter size of array\n");
   scanf("%d",&n);
   int arr[n];
   printf("Enter array elements \n");
   for(int i=0;i<n;i++){
    scanf("%d",&arr[i]);
   }
   int low,high;
   low = 0;
   high = n-1;
    // p = partition(arr,low,high);
   quicksort(arr,low,high);
   for(int i=0;i<n;i++){
        printf("%d",arr[i]);
   }
}