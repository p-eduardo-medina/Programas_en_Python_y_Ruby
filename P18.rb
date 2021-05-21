def increment_to_top(arr)
  total=0
	arr.each {|num| total += (arr.max-num).abs}
  return total
end


p increment_to_top([3, 4, 5])
