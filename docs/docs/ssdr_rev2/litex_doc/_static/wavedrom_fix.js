
    window.addEventListener('load', function() {
        function fixWaveDrom() {
            document.querySelectorAll('.wavedrom svg, svg[id^="svgcontent_"]').forEach(function(svg) {
                var w = svg.getAttribute('width');
                var h = svg.getAttribute('height');

                if (w && h && !svg.getAttribute('viewBox')) {
                    var wi = parseFloat(w);
                    var hi = parseFloat(h);
                    if (!isNaN(wi) && !isNaN(hi)) {
                        svg.setAttribute('viewBox', '0 0 ' + wi + ' ' + hi);
                    }
                }

                svg.style.display = 'block';
                svg.style.width = '100%';
                svg.style.maxWidth = '100%';
                svg.style.height = 'auto';

                var parent = svg.parentElement;
                if (parent) {
                    parent.style.display = 'block';
                    parent.style.width = '100%';
                    parent.style.maxWidth = '100%';
                    parent.style.overflowX = 'auto';
                }
            });
        }

        fixWaveDrom();

        var observer = new MutationObserver(function() {
            fixWaveDrom();
        });
        observer.observe(document.body, { childList: true, subtree: true });
    });
    