#include <stdio.h>
//fibonacci seq 
int fib(int num1){
    //base case
    if (num1 == 0) {
        return 0;
    }

    else if ((num1 == 1) || (num1 == 2)) {
        return 1;
    }
    else {
        return (fib(num1-1) + fib(num1-2));
    }
}

int main(void) {
    for(int i = 0; i <= 15; i++) {
        printf("%d\n", fib(i));
    }
    return 0;
}