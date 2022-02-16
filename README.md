# ros_dev

## 環境作成時
```
docker network create ros_dev_external
docker-compose build # Dockerfileを基にイメージを作成
docker-compose up -d # バックグラウンドでコンテナを起動
```

## コンテナへのアタッチ
```
cd <path>/ros_dev
./ros_shell # docker-compose exec ros_dev bash のエイリアス
```

## コンテナの停止
```
docker-compose stop
```

## コンテナの再起動
```
docker start
```
