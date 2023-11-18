#include<stdio.h>
#include"dec.h"
int main(){
    int n;
    scanf("%d",&n);
    int arr[n];
    for(int i=0;i<n;i++){
        scanf("%d",&arr[i]);
    }
    sel_sort(arr,n);
    display(arr,n);
    return 0;
}