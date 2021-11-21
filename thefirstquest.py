m = int(input())
if m%2==0:
    sum=(11+m)*((m-11+1)/2)
else:
    sum=(11+m-1)*((m-11)/2)+m
sum=int(sum)
print('sum =',sum)