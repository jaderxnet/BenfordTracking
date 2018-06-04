var http = require('http')
var url = require('url')
var fs = require('fs')
var path = require('path')

http.createServer(function (pedido, resposta) {
 
// Aqui vamos escrever o código do servidor que vai ser
// executado sempre que for feito um pedido
var caminho = url.parse(pedido.url).pathname;

if (caminho==='/') {
 var ficheiro = path.join(__dirname, 'public', caminho, 'index.html');
} else {
 var ficheiro = path.join(__dirname, 'public', caminho);
}
 
fs.readFile(ficheiro, function (erro, dados) {
if (erro) {
resposta.writeHead(404);
resposta.end();
} else {
resposta.end(dados);
}});

}).listen(8080, 'localhost', function () {
  console.log('--- O servidor arrancou –--');
});