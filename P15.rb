def reverse(str)
	str = (str.split).map {|element|
    if element.length >=5
      element = element.reverse
    else element = element
    end}
    return str.join(" ")
end


p reverse("Reverse")
p reverse("This is a typical sentence.")
