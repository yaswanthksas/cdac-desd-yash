#include<stdio.h>

int main(){
    int arr[] =  {5,10,2,1,4,9,15};
    int n = 7;
    int sea;
    int p = 0;
    scanf("%d",&sea);
    for(int i=0;i<n;i++){
        if(arr[i] == sea){
             p=1;
            printf("Element found\n");
        }
    }
    if(p == 0){
        printf("Element not found\n");
    }
    return 0;
}