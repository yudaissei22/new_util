# new_util

とりあえず、やってみたかんじのやつ

ボールを下投げする。

位置制御のP,Dゲインは、set-pdgain-rarm-roll ○で、強さの%を設定できる
throw ○、でボールを投げる速さを決められる。

課題
・滑らかに動いたため、見た目はしなっているが、運動連鎖(運動エネルギーが肩から手先まで伝わっていくことで、効率よく投擲動作をできる)とはなっていないくね？弱に、それをどうやって評価するのか？
・P,Dゲインを抜いたら、投げたあとはめっちゃふにゃふにゃになってしまう。。