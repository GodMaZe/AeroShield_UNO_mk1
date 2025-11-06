% muteia - aditive mutation of each individual of the population
%
%	Description:
%	The function mutates each individual of the population. The mutations are realized
%	by addition or subtraction of random real-numbers to the mutated genes. The 
%	absolute values of the added constants are limited by the vector Amp. 
%	Next the mutated strings are limited using boundaries defined in 
%	a two-row matrix Space. The first row of the matrix represents the lower 
%	boundaries and the second row represents the upper boundaries of corresponding 
%	genes.
%
%
%	Syntax: 
%
%	Newpop=muteia(Oldpop,rate,Amp,Space)
%
%	       Newpop - new, mutated population
%	       Oldpop - old population
%	       Amp   -  vector of absolute values of real-number boundaries
%	       Space  - matrix of gene boundaries in the form: 
%	                [real-number vector of lower limits of genes
%                        real-number vector of upper limits of genes];
%	       rate   - number of mutated genes, 1 =< rate =< 10
%

% I.Sekaj, 5/2000

function[Newpop]=muteia(Oldpop,N,Amps,Space)

[lpop,lstring]=size(Oldpop);

N=round(N);
if N<1 N=1; end;
if N>10 N=10; end;

Newpop=Oldpop;

for c=1:N
for r=1:lpop
s=ceil(rand*lstring);
Newpop(r,s)=Oldpop(r,s)+(2*rand-1)*Amps(s);
if Newpop(r,s)<Space(1,s) Newpop(r,s)=Space(1,s); end;
if Newpop(r,s)>Space(2,s) Newpop(r,s)=Space(2,s); end;
end;
end;

