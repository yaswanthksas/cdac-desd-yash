#include<stdio.h>

int main(){
    int row,col;
    printf("Enter row\n");
    scanf("%d",&row);
    printf("Enter column\n");
    scanf("%d",&col);
    int arr[row][col];
    int sum = 0;
    printf("Enter Elements :\n");
    for(int i=0;i<row;i++){
        for(int j=0;j<col;j++){
            scanf("%d",&arr[i][j]);
            if(i>j){
                sum += arr[i][j];
            }
        }
    }
    printf("sum is %d",sum);
    return 0;
}