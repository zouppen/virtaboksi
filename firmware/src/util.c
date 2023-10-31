char util_rot13(char c)
{
	if (c < 'a') return c;
	if (c > 'z') return c;
	c += 13;
	if (c > 'z') c -= 26;
	return c;
}
