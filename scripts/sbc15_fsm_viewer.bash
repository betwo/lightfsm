lastmsg=""
i="0"
while [[ 1 ]]; do
  msg=$(rostopic echo /fsm_state -n 1)
  if [[ "$msg" != "$lastmsg" ]]; then
    echo $msg | sed 's/^data:\(.*\)---$/\1/' | dot -Tpdf > state.pdf
    lastmsg=$msg
    if [[ $i == "0" ]]; then
      evince state.pdf --presentation &
      i="1"
    fi
  fi

done
