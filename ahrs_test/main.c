#include "ahrs.h"
#include <stdio.h>

ahrs_t ahrs;

int main() {
	printf("hello world\n");

	vector_t zeros = {0, 0, 1};
	update_ahrs(&ahrs, zeros, zeros, zeros, 1);
}
