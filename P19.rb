def array_of_multiples (num, length)
	tot = []
  length.times {|index| tot << num*(index+1) }
  return tot
end

def test_jackpot(result)
  value = 1
	3.times {|index|
    if result[index]==result[index+1]
      value*=1
    else value*=0
    end}
  return 1==value
end




def bitwise_and(n1, n2)
	n1,n2 =n1.to_s(2),n2.to_s(2)
  return (n1 and n2).to_i(2)
end

def bitwise_or(n1, n2)
  n1,n2 =n1.to_s(2),n2.to_s(2)
  return (n1 or n2).to_i(2)
end

def bitwise_xor(n1, n2)
  n1,n2 =n1.to_s(2),n2.to_s(2)
  return (n1 ^ n2).to_i(2)
end

p bitwise_and(7, 12)

p bitwise_or(7, 12)

p bitwise_xor(7, 12)
