P(V(1, do Move(5, 0) until Stop(0.1, "landmark[1]") then 2),
  V(2, do Move(3, 5) until Stop(0.1, "landmark[2]") then 3)
  ::V(3, do Move(5, 0) then 4)
  ::V(4, do Move(3, 10) then 5)
  ::V(5, if Visible("landmark[3]") then 6 else 7)
  ::V(6, do Say("Can I get a coffee please?") then 8)
  ::V(7, do Move(0, 0) then 8)
  ::V(8, end)
  ::nil)
