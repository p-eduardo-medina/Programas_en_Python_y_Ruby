def apocalyptic(n)
	if ((2**n).to_s).include?("666")
    return "Repent! "+(((2**n).to_s).index("666")).to_s+" days until the Apocalypse!"
  else return "Crisis averted. Resume sinning."
  end
end
=begin
"Repent! 6 days until the Apocalypse!"
p apocalyptic(109)

p apocalyptic(157)

p apocalyptic(175)

p apocalyptic(220)
=end
p apocalyptic(220)
