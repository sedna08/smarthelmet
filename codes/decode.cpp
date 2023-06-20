#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int main() {
    FILE *fp1, *fp2;
    double buffer[14];

    fp1 = fopen("data.bin", "r+b");
    fp2 = fopen("data.txt", "w+");
    while(!feof(fp1)){
        fread(buffer,sizeof(double),14,fp1);
        fprintf(fp2, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8],buffer[9],buffer[10],buffer[11],buffer[12],buffer[13]);
    }
    fclose(fp1);
    fclose(fp2);
}

